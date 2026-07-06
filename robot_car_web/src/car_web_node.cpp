#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <qing_robot_msgs/msg/ultrasonic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

using namespace std::chrono_literals;

struct CarStatus
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double linear_x = 0.0;
  double linear_y = 0.0;
  double angular_z = 0.0;
  double voltage = 0.0;
  double imu_wz = 0.0;
  double imu_ax = 0.0;
  double imu_ay = 0.0;
  bool charging = false;
  double charging_current = 0.0;
  int red_flag = 0;
  float ultrasonic[8] = {0.0f};
  rclcpp::Time odom_time;
  rclcpp::Time voltage_time;
  rclcpp::Time imu_time;
  rclcpp::Time ultrasonic_time;
  rclcpp::Time charging_time;
  rclcpp::Time map_pose_time;
  rclcpp::Time arm_time;
  rclcpp::Time arm_phase_time;
  rclcpp::Time task_time;
  bool have_map_pose = false;
  bool arm_moving = false;
  double arm_motion = 0.0;
  std::string arm_phase = "unknown";
  std::string arm_phase_label = "未连接";
  std::string task_json =
    R"JSON({"active":false,"state":"待分配","target":"护士站","target_room":"","medicine":"暂无","task_id":"--","task_type":"idle","task_name":"","stage":"idle","command_code":0,"message":"等待语音任务"})JSON";
};

struct MapStatus
{
  uint32_t width = 0;
  uint32_t height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  std::vector<int8_t> data;
  rclcpp::Time stamp;
  bool ready = false;
};

class CarWebNode : public rclcpp::Node
{
public:
  CarWebNode()
  : Node("car_web_node")
  {
    declare_parameter<std::string>("host", "0.0.0.0");
    declare_parameter<int>("port", 8080);
    get_parameter("host", host_);
    get_parameter("port", port_);

    status_.odom_time = now();
    status_.voltage_time = now();
    status_.imu_time = now();
    status_.ultrasonic_time = now();
    status_.charging_time = now();
    status_.map_pose_time = now();
    status_.arm_time = now();
    status_.arm_phase_time = now();
    status_.task_time = now();

    auto transient_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    auto telemetry_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", transient_qos, [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        map_.width = msg->info.width;
        map_.height = msg->info.height;
        map_.resolution = msg->info.resolution;
        map_.origin_x = msg->info.origin.position.x;
        map_.origin_y = msg->info.origin.position.y;
        map_.origin_yaw = yaw_from_quaternion(msg->info.origin.orientation);
        map_.data = msg->data;
        map_.stamp = now();
        map_.ready = true;
      });

    map_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", transient_qos, [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.x = msg->pose.pose.position.x;
        status_.y = msg->pose.pose.position.y;
        status_.yaw = yaw_from_quaternion(msg->pose.pose.orientation);
        status_.map_pose_time = now();
        status_.have_map_pose = true;
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom_combined", telemetry_qos, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!status_.have_map_pose) {
          status_.x = msg->pose.pose.position.x;
          status_.y = msg->pose.pose.position.y;
          status_.yaw = yaw_from_quaternion(msg->pose.pose.orientation);
        }
        status_.linear_x = msg->twist.twist.linear.x;
        status_.linear_y = msg->twist.twist.linear.y;
        status_.angular_z = msg->twist.twist.angular.z;
        status_.odom_time = now();
      });

    voltage_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/PowerVoltage", telemetry_qos, [this](std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.voltage = msg->data;
        status_.voltage_time = now();
      });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_raw", telemetry_qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.imu_wz = msg->angular_velocity.z;
        status_.imu_ax = msg->linear_acceleration.x;
        status_.imu_ay = msg->linear_acceleration.y;
        status_.imu_time = now();
      });

    ultrasonic_sub_ = create_subscription<qing_robot_msgs::msg::UltrasonicArray>(
      "/ultrasonic_array", telemetry_qos, [this](qing_robot_msgs::msg::UltrasonicArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.ultrasonic[0] = msg->front_left;
        status_.ultrasonic[1] = msg->front_center_left;
        status_.ultrasonic[2] = msg->front_center_right;
        status_.ultrasonic[3] = msg->front_right;
        status_.ultrasonic[4] = msg->rear_left;
        status_.ultrasonic[5] = msg->rear_center_left;
        status_.ultrasonic[6] = msg->rear_center_right;
        status_.ultrasonic[7] = msg->rear_right;
        status_.ultrasonic_time = now();
      });

    charging_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/robot_charging_flag", telemetry_qos, [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.charging = msg->data;
        status_.charging_time = now();
      });

    charging_current_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/robot_charging_current", telemetry_qos, [this](std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.charging_current = msg->data;
        status_.charging_time = now();
      });

    red_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/robot_red_flag", telemetry_qos, [this](std_msgs::msg::UInt8::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.red_flag = msg->data;
        status_.charging_time = now();
      });

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        double motion = 0.0;
        for (double v : msg->velocity) {
          motion = std::max(motion, std::abs(v));
        }
        std::lock_guard<std::mutex> lock(mutex_);
        status_.arm_motion = motion;
        status_.arm_moving = motion > 0.02;
        status_.arm_time = now();
      });

    arm_phase_sub_ = create_subscription<std_msgs::msg::String>(
      "/arm_phase", 10, [this](std_msgs::msg::String::SharedPtr msg) {
        const std::string phase = trim_copy(msg->data);
        std::lock_guard<std::mutex> lock(mutex_);
        status_.arm_phase = phase.empty() ? "unknown" : phase;
        status_.arm_phase_label = arm_phase_label(status_.arm_phase);
        status_.arm_phase_time = now();
        status_.arm_time = status_.arm_phase_time;
      });

    task_sub_ = create_subscription<std_msgs::msg::String>(
      "/voice_nav_task", transient_qos, [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.task_json = json_object_or_task_state(msg->data);
        status_.task_time = now();
      });

    delivery_record_sub_ = create_subscription<std_msgs::msg::String>(
      "/medicine_delivery_record", 10, [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        delivery_uploads_.push_back(delivery_upload_payload(msg->data));
        if (delivery_uploads_.size() > 120) {
          delivery_uploads_.erase(delivery_uploads_.begin());
        }
      });

    server_thread_ = std::thread([this]() { serve(); });
    RCLCPP_INFO(get_logger(), "car web dashboard listening on http://%s:%d", host_.c_str(), port_);
  }

  ~CarWebNode() override
  {
    running_ = false;
    if (server_fd_ >= 0) {
      shutdown(server_fd_, SHUT_RDWR);
      close(server_fd_);
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

private:
  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  double age_seconds(const rclcpp::Time &time) const
  {
    return std::max(0.0, (now() - time).seconds());
  }

  std::string fmt(double value, int precision = 3) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
  }

  std::string json_escape(const std::string & value) const
  {
    std::ostringstream out;
    for (const unsigned char c : value) {
      switch (c) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\b': out << "\\b"; break;
        case '\f': out << "\\f"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
          if (c < 0x20) {
            out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                << static_cast<int>(c) << std::dec << std::setfill(' ');
          } else {
            out << static_cast<char>(c);
          }
          break;
      }
    }
    return out.str();
  }

  static std::string trim_copy(const std::string & value)
  {
    auto begin = value.begin();
    while (begin != value.end() && std::isspace(static_cast<unsigned char>(*begin))) {
      ++begin;
    }
    auto end = value.end();
    while (end != begin && std::isspace(static_cast<unsigned char>(*(end - 1)))) {
      --end;
    }
    return std::string(begin, end);
  }

  static bool looks_like_json_object(const std::string & value)
  {
    const std::string trimmed = trim_copy(value);
    return trimmed.size() >= 2 && trimmed.front() == '{' && trimmed.back() == '}';
  }

  static long long wall_time_millis()
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  }

  static std::string arm_phase_label(const std::string & phase)
  {
    if (phase == "idle") return "空闲";
    if (phase == "observe") return "观察位";
    if (phase == "pick_down") return "夹取下探";
    if (phase == "pick_up") return "夹取抬起";
    if (phase == "place") return "放置位";
    return "未连接";
  }

  std::string json_object_or_task_state(const std::string & value) const
  {
    const std::string trimmed = trim_copy(value);
    if (!trimmed.empty() && trimmed.front() == '{' && trimmed.back() == '}') {
      return trimmed;
    }
    const std::string state = trimmed.empty() ? "待分配" : trimmed;
    return "{\"active\":false,\"state\":\"" + json_escape(state) +
      "\",\"target\":\"护士站\",\"target_room\":\"\",\"medicine\":\"暂无\","
      "\"task_id\":\"--\",\"task_type\":\"text\",\"task_name\":\"\","
      "\"stage\":\"unknown\",\"command_code\":0,\"message\":\"\"}";
  }

  std::string delivery_upload_payload(const std::string & record) const
  {
    const auto timestamp = wall_time_millis();
    const std::string trimmed = trim_copy(record);
    const std::string value_json = looks_like_json_object(trimmed)
      ? trimmed
      : "{\"text\":\"" + json_escape(trimmed) + "\"}";

    std::ostringstream out;
    out << "{"
        << "\"id\":\"" << timestamp << "\","
        << "\"version\":\"1.0\","
        << "\"params\":{"
        << "\"medicine_delivery\":{"
        << "\"value\":" << value_json << ","
        << "\"time\":" << timestamp
        << "}"
        << "}"
        << "}";
    return out.str();
  }

  std::vector<std::string> tail_lines(const std::string & path, std::size_t limit) const
  {
    std::ifstream input(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
      lines.push_back(line);
      if (lines.size() > limit) {
        lines.erase(lines.begin());
      }
    }
    return lines;
  }

  std::string command_output(const std::string & cmd) const
  {
    std::array<char, 256> buffer{};
    std::string result;
    FILE * pipe = popen(cmd.c_str(), "r");
    if (pipe == nullptr) {
      return "";
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
      result += buffer.data();
    }
    pclose(pipe);
    return result;
  }

  static std::string payload_from_log_line(const std::string & line)
  {
    const std::string marker = "payload=";
    const auto pos = line.find(marker);
    if (pos == std::string::npos) {
      return "";
    }
    return trim_copy(line.substr(pos + marker.size()));
  }

  std::string json_status()
  {
    CarStatus s;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      s = status_;
    }

    std::ostringstream out;
    out << "{"
        << "\"pose\":{\"x\":" << fmt(s.x) << ",\"y\":" << fmt(s.y)
        << ",\"yaw\":" << fmt(s.yaw) << "},"
        << "\"velocity\":{\"linear_x\":" << fmt(s.linear_x)
        << ",\"linear_y\":" << fmt(s.linear_y)
        << ",\"angular_z\":" << fmt(s.angular_z) << "},"
        << "\"power\":{\"voltage\":" << fmt(s.voltage, 2)
        << ",\"charging\":" << (s.charging ? "true" : "false")
        << ",\"charging_current\":" << fmt(s.charging_current, 2)
        << ",\"red_flag\":" << s.red_flag << "},"
        << "\"imu\":{\"angular_z\":" << fmt(s.imu_wz)
        << ",\"accel_x\":" << fmt(s.imu_ax)
        << ",\"accel_y\":" << fmt(s.imu_ay) << "},"
        << "\"arm\":{\"moving\":" << (s.arm_moving ? "true" : "false")
        << ",\"motion\":" << fmt(s.arm_motion, 3)
        << ",\"phase\":\"" << json_escape(s.arm_phase) << "\""
        << ",\"phase_label\":\"" << json_escape(s.arm_phase_label) << "\""
        << ",\"phase_age\":" << fmt(age_seconds(s.arm_phase_time), 1) << "},"
        << "\"task\":" << s.task_json << ","
        << "\"ultrasonic\":[";
    for (int i = 0; i < 8; ++i) {
      if (i > 0) out << ",";
      out << fmt(s.ultrasonic[i], 3);
    }
    out << "],\"age\":{"
        << "\"odom\":" << fmt(age_seconds(s.odom_time), 1) << ","
        << "\"voltage\":" << fmt(age_seconds(s.voltage_time), 1) << ","
        << "\"imu\":" << fmt(age_seconds(s.imu_time), 1) << ","
        << "\"ultrasonic\":" << fmt(age_seconds(s.ultrasonic_time), 1) << ","
        << "\"map_pose\":" << fmt(age_seconds(s.map_pose_time), 1) << ","
        << "\"arm\":" << fmt(age_seconds(s.arm_time), 1) << ","
        << "\"task\":" << fmt(age_seconds(s.task_time), 1) << ","
        << "\"charging\":" << fmt(age_seconds(s.charging_time), 1)
        << "}}";
    return out.str();
  }

  std::string json_map()
  {
    MapStatus m;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      m = map_;
    }

    std::ostringstream out;
    out << "{\"ready\":" << (m.ready ? "true" : "false")
        << ",\"width\":" << m.width
        << ",\"height\":" << m.height
        << ",\"resolution\":" << fmt(m.resolution, 4)
        << ",\"origin\":{\"x\":" << fmt(m.origin_x)
        << ",\"y\":" << fmt(m.origin_y)
        << ",\"yaw\":" << fmt(m.origin_yaw) << "}"
        << ",\"data\":[";
    for (size_t i = 0; i < m.data.size(); ++i) {
      if (i > 0) out << ",";
      out << static_cast<int>(m.data[i]);
    }
    out << "]}";
    return out.str();
  }

  std::string json_cloud()
  {
    const std::string log_path = "/home/sunrise/qian_sai/onenet_bridge.runtime.log";
    const auto logs = tail_lines(log_path, 400);
    std::vector<std::string> local_uploads;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      local_uploads = delivery_uploads_;
    }
    const std::string tcp = command_output(
      "ss -tan | awk '$4 ~ /:1883$/ || $5 ~ /:1883$/ || $4 ~ /:8883$/ || $5 ~ /:8883$/ {print}'");

    bool saw_connected = false;
    bool saw_lost_after_connected = false;
    std::string last_upload;
    std::string last_reply;
    std::string last_event;
    std::string last_error;
    std::string pending_log_upload;
    std::vector<std::string> uploads;
    auto remember_upload = [&](const std::string & upload) {
      if (upload.empty()) {
        return;
      }
      uploads.push_back(upload);
      if (uploads.size() > 120) {
        uploads.erase(uploads.begin());
      }
      last_upload = upload;
    };
    for (const auto & line : logs) {
      if (line.find("OneNET MQTT connected") != std::string::npos) {
        saw_connected = true;
        saw_lost_after_connected = false;
      }
      if (line.find("Uploading delivery record") != std::string::npos) {
        pending_log_upload = payload_from_log_line(line);
      }
      if (line.find("connection lost") != std::string::npos ||
          line.find("Failed to upload") != std::string::npos)
      {
        saw_lost_after_connected = true;
        last_error = line;
        pending_log_upload.clear();
      }
      if (line.find("delivery event report") != std::string::npos ||
          line.find("Uploaded delivery record") != std::string::npos)
      {
        last_event = line;
      }
      if (line.find("\"code\":200") != std::string::npos) {
        last_event = line;
        last_reply = line;
        if (line.find("delivery event report") != std::string::npos) {
          remember_upload(pending_log_upload);
          pending_log_upload.clear();
        }
      }
    }
    if (uploads.empty()) {
      for (const auto & upload : local_uploads) {
        remember_upload(upload);
      }
    }

    const bool tcp_established = tcp.find("ESTAB") != std::string::npos;
    const bool online = tcp_established && saw_connected && !saw_lost_after_connected;
    const std::string state = online ? "online" : (tcp_established ? "connecting" : "offline");

    std::ostringstream out;
    out << "{"
        << "\"state\":\"" << state << "\","
        << "\"online\":" << (online ? "true" : "false") << ","
        << "\"tcp_established\":" << (tcp_established ? "true" : "false") << ","
        << "\"endpoint\":\"mqtts.heclouds.com:1883\","
        << "\"tcp\":\"" << json_escape(tcp) << "\","
        << "\"last_upload\":\"" << json_escape(last_upload) << "\","
        << "\"last_reply\":\"" << json_escape(last_reply) << "\","
        << "\"last_event\":\"" << json_escape(last_event) << "\","
        << "\"last_error\":\"" << json_escape(last_error) << "\","
        << "\"uploads\":[";
    for (size_t i = 0; i < uploads.size(); ++i) {
      if (i > 0) out << ",";
      out << "\"" << json_escape(uploads[i]) << "\"";
    }
    out << "]}";
    return out.str();
  }

  std::string start_navigation()
  {
    static std::atomic<bool> nav_starting{false};
    return start_command(
      nav_starting,
      "送药系统正在启动",
      "已发送送药系统启动命令",
      "bash -lc 'source /home/sunrise/qian_sai/install/setup.bash && nohup ros2 launch voice_nav_control voice_nav_launch.py > /home/sunrise/qian_sai/voice_nav_launch.log 2>&1 &'");
  }

  std::string start_command(
    std::atomic<bool> &starting,
    const std::string &starting_message,
    const std::string &started_message,
    const std::string &cmd)
  {
    if (starting.exchange(true)) {
      return "{\"ok\":true,\"message\":\"" + starting_message + "\"}";
    }
    std::thread([&starting, cmd]() {
      (void)std::system(cmd.c_str());
      starting = false;
    }).detach();
    return "{\"ok\":true,\"message\":\"" + started_message + "\"}";
  }

  std::string start_base()
  {
    static std::atomic<bool> starting{false};
    return start_command(
      starting,
      "底盘正在启动",
      "已发送底盘启动命令",
      "bash -lc 'source /home/sunrise/qian_sai/install/setup.bash && nohup ros2 launch communication_base base_serial.launch.py > /home/sunrise/qian_sai/base_serial.launch.log 2>&1 &'");
  }

  std::string start_lidar()
  {
    static std::atomic<bool> starting{false};
    return start_command(
      starting,
      "雷达正在启动",
      "已发送雷达启动命令",
      "bash -lc 'source /home/sunrise/qian_sai/install/setup.bash && nohup ros2 launch oradar_lidar ms200_scan.launch.py port_name:=/dev/oradar_lidar frame_id:=laser scan_topic:=/scan publish_tf:=true > /home/sunrise/qian_sai/oradar_lidar.launch.log 2>&1 &'");
  }

  std::string start_mapping()
  {
    static std::atomic<bool> starting{false};
    return start_command(
      starting,
      "建图正在启动",
      "已发送建图启动命令",
      "bash -lc 'source /home/sunrise/qian_sai/install/setup.bash && nohup ros2 launch slam_cartgorpher cartographer_mapping.launch.py start_robot:=true start_lidar:=true start_rviz:=false > /home/sunrise/qian_sai/cartographer_mapping.launch.log 2>&1 &'");
  }

  std::string save_map()
  {
    static std::atomic<bool> starting{false};
    return start_command(
      starting,
      "地图正在保存",
      "已发送地图保存命令",
      "bash -lc 'source /home/sunrise/qian_sai/install/setup.bash && nohup ros2 launch slam_cartgorpher save_map.launch.py > /home/sunrise/qian_sai/save_map.launch.log 2>&1 &'");
  }

  std::string start_keyboard_control()
  {
    static std::atomic<bool> starting{false};
    return start_command(
      starting,
      "键盘控制正在启动",
      "已打开键盘控制窗口",
      "bash -lc 'source /home/sunrise/qian_sai/install/setup.bash && nohup ros2 launch key_control key_control.launch.py start_base:=false > /home/sunrise/qian_sai/key_control.launch.log 2>&1 &'" );
  }

  std::string html_page() const
  {
    return R"HTML(<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>青鸾守护</title>
  <style>
*{box-sizing:border-box;margin:0;padding:0}:root{--bg:#f0f2f5;--card:#fff;--border:rgba(0,0,0,.07);--border2:rgba(0,0,0,.03);--text:#1e293b;--text2:#64748b;--text3:#94a3b8;--accent:#0d9488;--accent2:#14b8a6;--accent-l:rgba(13,148,136,.07);--green:#10b981;--green-l:rgba(16,185,129,.08);--red:#ef4444;--red-l:rgba(239,68,68,.06);--amber:#f59e0b;--amber-l:rgba(245,158,11,.08);--blue:#3b82f6;--blue-l:rgba(59,130,246,.06);--r:10px;--rs:6px}html,body{height:100vh;overflow:hidden;background:var(--bg);color:var(--text);font-family:'Inter',system-ui,-apple-system,'Segoe UI','Microsoft YaHei',sans-serif;font-size:12px;-webkit-font-smoothing:antialiased}button,textarea{font:inherit}.app{display:flex;flex-direction:column;height:100vh}.hdr{height:38px;display:flex;align-items:center;justify-content:space-between;padding:0 10px;background:#fff;border-bottom:1px solid var(--border);flex-shrink:0;z-index:10}.brand{display:flex;align-items:center;gap:6px}.logo{width:24px;height:24px;border-radius:6px;display:grid;place-items:center;background:linear-gradient(135deg,#0d9488,#06b6d4);box-shadow:0 2px 8px rgba(13,148,136,.25)}.logo svg{width:15px;height:15px;color:#fff}.brand h1{font-size:14px;font-weight:700;color:var(--text)}.brand em{font-size:11px;color:var(--text3);font-style:normal;margin-left:4px}.hdr-r{display:flex;align-items:center;gap:6px}.hdr-clock{font-size:11px;color:var(--text2);font-weight:600;font-variant-numeric:tabular-nums;display:flex;align-items:center;gap:5px;padding:3px 10px;background:#f8fafc;border-radius:14px;border:1px solid var(--border2)}.hdr-clock .hc-time{color:var(--text);font-weight:800;font-size:12px}.hdr-clock .hc-date{color:var(--text3);font-size:10px}.conn-quality{display:flex;align-items:center;gap:3px}.conn-bar{width:3px;border-radius:1px;background:var(--green);transition:height .3s}.btn{height:26px;padding:0 14px;border:none;border-radius:14px;font-size:11px;font-weight:600;cursor:pointer;transition:all .2s}.bp{color:#fff;background:linear-gradient(135deg,#0d9488,#06b6d4);box-shadow:0 2px 8px rgba(13,148,136,.3)}.bp:hover{box-shadow:0 4px 14px rgba(13,148,136,.4);transform:translateY(-1px)}.bg{color:var(--text2);background:#fff;border:1px solid var(--border)}.bg:hover{border-color:var(--accent);color:var(--accent)}.badge{display:inline-flex;align-items:center;gap:3px;padding:3px 10px;border-radius:14px;font-size:10px;font-weight:600}.badge-on{background:var(--green-l);color:var(--green)}.badge-off{background:var(--red-l);color:var(--red)}.dot{width:6px;height:6px;border-radius:50%;background:currentColor;animation:pulse 2s infinite}@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}.main{flex:1;display:grid;grid-template-columns:2.8fr .85fr 1.15fr;grid-template-rows:auto 1fr;gap:3px;padding:3px 4px;min-height:0}.strip{grid-column:1/-1;display:grid;grid-template-columns:1fr 1fr 1.5fr 1.5fr 1.8fr;gap:3px}.pnl{background:var(--card);border:1px solid var(--border);border-radius:8px;box-shadow:0 1px 3px rgba(0,0,0,.03);overflow:hidden;display:flex;flex-direction:column;position:relative}.pnl-h{font-size:11px;font-weight:700;color:var(--text2);text-transform:uppercase;letter-spacing:.5px;padding:5px 8px 4px;display:flex;align-items:center;gap:5px;flex-shrink:0;background:linear-gradient(180deg,rgba(0,0,0,.015),transparent);border-bottom:1px solid var(--border2)}.pnl-h svg{width:14px;height:14px;opacity:.6}.pnl-b{flex:1;padding:6px 10px 8px;min-height:0;display:flex;flex-direction:column;overflow-y:auto;justify-content:space-evenly}.pnl-b::-webkit-scrollbar{width:2px}.pnl-b::-webkit-scrollbar-thumb{background:#ddd;border-radius:2px}.pnl::before{content:'';position:absolute;top:0;left:0;bottom:0;width:3px;border-radius:var(--r) 0 0 var(--r)}.pnl.c1::before{background:linear-gradient(180deg,#0d9488,#06b6d4)}.pnl.c2::before{background:linear-gradient(180deg,#3b82f6,#6366f1)}.pnl.c3::before{background:linear-gradient(180deg,#0d9488,#10b981)}.pnl.c4::before{background:linear-gradient(180deg,#10b981,#06b6d4)}.pnl.c5::before{background:linear-gradient(180deg,#06b6d4,#14b8a6)}.sc{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:5px 8px;box-shadow:0 1px 3px rgba(0,0,0,.03);position:relative;overflow:hidden;display:flex;flex-direction:column;justify-content:center;animation:fadeUp .4s ease both}.sc:nth-child(1){animation-delay:.05s}.sc:nth-child(2){animation-delay:.1s}.sc:nth-child(3){animation-delay:.15s}.sc:nth-child(4){animation-delay:.2s}.sc:nth-child(5){animation-delay:.25s}.sc::after{content:'';position:absolute;bottom:0;left:10%;right:10%;height:2px;border-radius:1px}.sc:nth-child(1)::after{background:linear-gradient(90deg,transparent,#0d9488,transparent)}.sc:nth-child(2)::after{background:linear-gradient(90deg,transparent,#64748b,transparent)}.sc:nth-child(3)::after{background:linear-gradient(90deg,transparent,#0d9488,transparent)}.sc:nth-child(4)::after{background:linear-gradient(90deg,transparent,#10b981,transparent)}.sc:nth-child(5)::after{background:linear-gradient(90deg,transparent,#06b6d4,transparent)}.sc-label{font-size:10px;font-weight:700;color:var(--text3);text-transform:uppercase;letter-spacing:.5px;margin-bottom:2px}.sc-v{font-size:20px;font-weight:800;line-height:1.1}.sc-sub{font-size:11px;color:var(--text3);margin-top:1px}.sc-icon{width:28px;height:28px;border-radius:8px;display:grid;place-items:center;flex-shrink:0}.sc-icon svg{width:15px;height:15px}.sc-row{display:flex;align-items:center;gap:6px}.sc-spark{display:flex;align-items:flex-end;gap:1px;height:16px}.sc-spark span{width:3px;border-radius:1px;background:var(--accent);opacity:.5;transition:height .3s}.tag{display:inline-flex;padding:2px 8px;border-radius:4px;font-size:10px;font-weight:600;margin-top:2px}.tw{background:var(--amber-l);color:var(--amber)}.tok{background:var(--green-l);color:var(--green)}.te{background:var(--red-l);color:var(--red)}.ti{background:var(--blue-l);color:var(--blue)}.gm{display:flex;align-items:center;gap:8px}.g-svg{width:76px;height:44px;flex-shrink:0}.g-info .gv{font-size:22px;font-weight:800;line-height:1}.g-info .gu{font-size:10px;color:var(--text3);margin-top:1px}.arm-r{display:flex;align-items:center;gap:8px}.arm-box{width:72px;height:40px;border-radius:var(--rs);background:#f8fafc;border:1px solid var(--border2);overflow:hidden;flex-shrink:0}.arm-box canvas{width:100%;height:100%}.arm-d{flex:1;font-size:11px}.arm-d div{display:flex;justify-content:space-between;padding:1px 0}.arm-d .al{color:var(--text3)}.arm-d .av{font-weight:600}.av.bad{color:var(--red)}.av.ok{color:var(--green)}.bar-bg{height:3px;border-radius:2px;background:#f1f5f9;overflow:hidden;margin-top:3px}.bar-fg{height:100%;border-radius:2px;background:linear-gradient(90deg,var(--accent),var(--green));transition:width .5s}.map-area{flex:1;min-height:0;border-radius:var(--rs);overflow:hidden;position:relative;background:linear-gradient(135deg,#e8ecf1,#f0f2f5);border:1px solid var(--border2)}.map-area canvas{width:100%;height:100%;display:block}.map-ph{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center;color:var(--text3);pointer-events:none}.map-ph svg{width:28px;height:28px;opacity:.15;margin-bottom:3px}.map-ph .mt{font-size:12px;font-weight:600}.map-ph .ms{font-size:10px;opacity:.6;margin-top:1px}.map-ft{display:flex;align-items:center;justify-content:space-between;padding:2px 6px;flex-shrink:0}.map-badge{padding:2px 8px;border-radius:10px;font-size:10px;font-weight:600;background:var(--accent-l);color:var(--accent);border:1px solid rgba(13,148,136,.12)}.map-st{font-size:10px;color:var(--text3)}.dr{display:flex;justify-content:space-between;align-items:center;padding:6px 0;font-size:12px;border-bottom:1px solid var(--border2)}.dr:last-child{border-bottom:none}.dr .dl{color:var(--text3);font-size:11px;display:flex;align-items:center;gap:5px}.dr .dl::before{content:'';width:6px;height:6px;border-radius:50%;flex-shrink:0}.dr .dv{font-weight:700;font-variant-numeric:tabular-nums;font-size:14px}.dr .dv.ok{color:var(--green)}.dr .dv.wn{color:var(--amber)}.dr .dv.bad{color:var(--red)}.tv.empty{color:var(--text3);font-style:italic;font-weight:400}.sec{font-size:10px;font-weight:700;color:var(--text3);text-transform:uppercase;letter-spacing:.5px;padding:2px 0 2px;border-bottom:1px solid var(--border2);margin-bottom:2px;margin-top:3px}.sec:first-child{margin-top:0}.pnl.c3 .dl::before{background:linear-gradient(135deg,#0d9488,#10b981)}.pnl.c4 .dl::before{background:linear-gradient(135deg,#10b981,#06b6d4)}.pnl.c5 .dl::before{background:linear-gradient(135deg,#06b6d4,#14b8a6)}.pnl.c2 .dl::before{background:linear-gradient(135deg,#3b82f6,#6366f1)}.pnl.c1 .dl::before{background:linear-gradient(135deg,#0d9488,#06b6d4)}.mini-bar{height:3px;border-radius:2px;background:#f1f5f9;overflow:hidden;margin-top:2px;flex-shrink:0}.mini-bar-fg{height:100%;border-radius:2px;transition:width .6s ease}.si{display:inline-flex;align-items:center;gap:4px;padding:2px 8px;border-radius:10px;font-size:10px;font-weight:600}.si-warn{background:var(--amber-l);color:var(--amber);border:1px solid rgba(245,158,11,.12)}.si-ok{background:var(--green-l);color:var(--green);border:1px solid rgba(16,185,129,.12)}.si-info{background:var(--blue-l);color:var(--blue);border:1px solid rgba(59,130,246,.12)}.si-err{background:var(--red-l);color:var(--red);border:1px solid rgba(239,68,68,.12)}.cloud-ep{font-size:10px;color:var(--accent);font-family:'Cascadia Code','Fira Code',monospace;padding:3px 8px;background:var(--accent-l);border-radius:4px;word-break:break-all;margin:2px 0;border:1px solid rgba(13,148,136,.08)}.cloud-log{width:100%;flex:1;min-height:0;resize:none;border:1px solid var(--border2);border-radius:var(--rs);background:#fafbfc;color:var(--text2);padding:5px 8px;font-size:10px;line-height:1.5;outline:none;font-family:'Cascadia Code','Fira Code',monospace}.cloud-events{display:flex;flex-direction:column;gap:2px}.cloud-ev{display:flex;align-items:center;gap:5px;padding:3px 6px;border-radius:4px;font-size:10px;background:#f8fafc;border:1px solid var(--border2);transition:all .2s}.cloud-ev:hover{border-color:var(--accent);background:var(--accent-l)}.cloud-ev .ev-dot{width:5px;height:5px;border-radius:50%;flex-shrink:0}.cloud-ev .ev-time{font-size:9px;color:var(--text3);margin-left:auto;font-variant-numeric:tabular-nums}.cloud-stats{display:flex;gap:3px}.cloud-stat{flex:1;text-align:center;padding:3px 0;border-radius:4px;background:#f8fafc;border:1px solid var(--border2)}.cloud-stat .cs-v{font-size:13px;font-weight:800;line-height:1.2}.cloud-stat .cs-l{font-size:8px;color:var(--text3);font-weight:600}.upload-item{display:flex;align-items:center;gap:5px;padding:3px 6px;border-radius:5px;background:#f8fafc;border:1px solid var(--border2);font-size:10px;transition:all .15s}.upload-item:hover{border-color:var(--accent);background:var(--accent-l)}.upload-item .ui-icon{width:20px;height:20px;border-radius:5px;display:grid;place-items:center;flex-shrink:0}.upload-item .ui-info{flex:1;min-width:0}.upload-item .ui-name{font-weight:600;color:var(--text);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.upload-item .ui-meta{font-size:9px;color:var(--text3)}.bbar{height:32px;display:flex;align-items:center;gap:5px;padding:0 8px;background:#fff;border-top:1px solid var(--border);flex-shrink:0}.cmds{flex:1;display:flex;gap:5px;overflow-x:auto;scrollbar-width:none}.cmds::-webkit-scrollbar{display:none}.cb{flex-shrink:0;height:24px;padding:0 10px;border:1px solid var(--border);border-radius:var(--rs);background:#fff;color:var(--text2);font-size:11px;font-weight:600;cursor:pointer;display:flex;align-items:center;gap:4px;transition:all .15s;white-space:nowrap}.cb:hover{border-color:var(--accent);color:var(--accent);background:var(--accent-l)}.cb:active{transform:scale(.96)}.cb.acc{border-color:rgba(13,148,136,.15);color:var(--accent);background:var(--accent-l)}.cb small{font-size:9px;color:var(--text3)}.cb svg{width:12px;height:12px;opacity:.6;flex-shrink:0}.cb:hover svg{opacity:1}.cb.loading{opacity:.4;pointer-events:none}.bst{flex-shrink:0;font-size:11px;color:var(--text3);padding-left:8px;border-left:1px solid var(--border);white-space:nowrap}.snackbar-container{position:fixed;bottom:44px;left:50%;transform:translateX(-50%);z-index:100;display:flex;flex-direction:column;gap:8px;align-items:center;pointer-events:none}.snackbar{min-width:280px;padding:7px 18px;border-radius:8px;background:rgba(30,41,59,.92);backdrop-filter:blur(8px);color:#fff;font-size:11px;font-weight:500;box-shadow:0 4px 16px rgba(0,0,0,.12);animation:si .3s ease;pointer-events:auto}.snackbar.out{opacity:0;transform:translateY(20px);transition:all .3s}@keyframes si{from{opacity:0;transform:translateX(-50%) translateY(8px)}to{opacity:1;transform:translateX(-50%)}}.ripple{position:absolute;border-radius:50%;background:rgba(13,148,136,.08);transform:scale(0);animation:rp .4s ease-out forwards;pointer-events:none}@keyframes rp{to{transform:scale(3);opacity:0}}@keyframes imu_spin{from{transform:rotate(0deg)}to{transform:rotate(360deg)}}@keyframes fadeUp{from{opacity:0;transform:translateY(6px)}to{opacity:1;transform:translateY(0)}}.hidden{display:none!important}
  </style>
</head>
<body>
<div class="app">
<header class="hdr">
  <div class="brand">
    <div class="logo"><svg viewBox="0 0 24 24" fill="none"><path d="M12 3l7 4v6c0 4.2-2.8 7.4-7 8-4.2-.6-7-3.8-7-8V7l7-4z" stroke="currentColor" stroke-width="2.5"/><path d="M12 7v8M8 11h8" stroke="currentColor" stroke-width="2.5" stroke-linecap="round"/></svg></div>
    <h1>青鸾守护</h1><em>病区物流机器人</em>
    <span style="font-size:9px;color:var(--text3);margin-left:6px;padding:2px 6px;background:#f1f5f9;border-radius:4px;font-weight:600">v2.4.1</span>
  </div>
  <div class="hdr-r">
    <div class="hdr-clock"><svg viewBox="0 0 24 24" width="12" height="12" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><path d="M12 6v6l4 2"/></svg><span class="hc-time" id="hdr_time">--:--:--</span><span class="hc-date" id="hdr_date">----</span></div>
    <div class="conn-quality" title="连接质量"><div class="conn-bar" style="height:4px"></div><div class="conn-bar" style="height:7px"></div><div class="conn-bar" style="height:10px"></div><div class="conn-bar" style="height:13px;opacity:.3"></div><span style="font-size:10px;color:var(--text3);margin-left:2px" id="conn_label">WiFi</span></div>
    <div style="width:1px;height:18px;background:var(--border);margin:0 2px"></div>
    <button class="btn bp" id="start_nav">启动送药系统</button>
    <button class="btn bg" id="ctrl_car">控制小车</button>
    <span class="badge badge-on" id="system_state"><span class="dot"></span>连接中</span>
    <span class="badge badge-off hidden" id="cloud_state"><span class="dot"></span>云平台</span>
  </div>
</header>
<div class="main">
<div class="strip">
  <div class="sc"><div class="sc-row"><div class="sc-icon" style="background:linear-gradient(135deg,rgba(13,148,136,.1),rgba(16,185,129,.08))"><svg viewBox="0 0 24 24" fill="none" stroke="#0d9488" stroke-width="2"><path d="M12 2v4M12 18v4M4.93 4.93l2.83 2.83M16.24 16.24l2.83 2.83M2 12h4M18 12h4M4.93 19.07l2.83-2.83M16.24 7.76l2.83-2.83"/></svg></div><div style="flex:1"><div class="sc-label">运行状态</div><div class="sc-v" id="motion_state" style="color:var(--text2);font-size:17px">待机</div><div class="tag tw" id="fresh_state" style="font-size:9px;padding:1px 6px">--</div></div><div class="sc-spark" id="run_spark"><span style="height:30%"></span><span style="height:60%"></span><span style="height:45%"></span><span style="height:80%"></span><span style="height:55%"></span><span style="height:70%"></span><span style="height:40%"></span></div></div></div>
  <div class="sc"><div class="sc-row"><div class="sc-icon" style="background:linear-gradient(135deg,rgba(100,116,139,.08),rgba(71,85,105,.06))"><svg viewBox="0 0 24 24" fill="none" stroke="#64748b" stroke-width="2"><path d="M5 12h14"/><path d="M12 5l7 7-7 7"/><rect x="3" y="8" width="4" height="8" rx="1" fill="rgba(100,116,139,.15)"/></svg></div><div style="flex:1"><div class="sc-label">回充状态</div><div class="sc-v" id="charging" style="color:var(--text2);font-size:17px">未充电</div><div class="sc-sub">电流 <b id="current">0</b>A · 红外 <b id="red">0</b></div></div><div style="width:28px;height:28px;border-radius:50%;background:#f1f5f9;display:grid;place-items:center;flex-shrink:0"><svg viewBox="0 0 24 24" width="14" height="14" fill="none" stroke="#94a3b8" stroke-width="2"><path d="M13 2L3 14h9l-1 8 10-12h-9l1-8z"/></svg></div></div></div>
  <div class="sc"><div class="sc-label">车速</div><div class="gm"><svg class="g-svg" viewBox="0 0 140 80"><defs><linearGradient id="gg1" x1="0" y1="0" x2="1" y2="0"><stop offset="0%" stop-color="#10b981"/><stop offset="60%" stop-color="#f59e0b"/><stop offset="100%" stop-color="#ef4444"/></linearGradient></defs><path d="M 15 72 A 55 55 0 0 1 125 72" fill="none" stroke="#e5e7eb" stroke-width="10" stroke-linecap="round"/><path d="M 15 72 A 55 55 0 0 1 125 72" fill="none" stroke="url(#gg1)" stroke-width="10" stroke-linecap="round" stroke-dasharray="173" stroke-dashoffset="173" id="speed_arc"/><line x1="70" y1="72" x2="70" y2="28" stroke="#1e293b" stroke-width="2" stroke-linecap="round" transform="rotate(-90,70,72)" id="speed_needle" style="transition:transform .5s"/><circle cx="70" cy="72" r="3" fill="#1e293b"/><text x="18" y="76" font-size="7" fill="#94a3b8">0</text><text x="66" y="16" font-size="7" fill="#f59e0b" text-anchor="middle">1.0</text><text x="118" y="76" font-size="7" fill="#ef4444">2.0</text></svg><div class="g-info"><div class="gv"><span id="speed_val">0.00</span></div><div class="gu">m/s</div></div></div></div>
  <div class="sc"><div class="sc-label">电量</div><div class="gm"><svg class="g-svg" viewBox="0 0 140 80"><defs><linearGradient id="gg2" x1="0" y1="0" x2="1" y2="0"><stop offset="0%" stop-color="#ef4444"/><stop offset="30%" stop-color="#f59e0b"/><stop offset="70%" stop-color="#10b981"/><stop offset="100%" stop-color="#0d9488"/></linearGradient></defs><path d="M 15 72 A 55 55 0 0 1 125 72" fill="none" stroke="#e5e7eb" stroke-width="10" stroke-linecap="round"/><path d="M 15 72 A 55 55 0 0 1 125 72" fill="none" stroke="url(#gg2)" stroke-width="10" stroke-linecap="round" stroke-dasharray="173" stroke-dashoffset="173" id="battery_arc"/><line x1="70" y1="72" x2="70" y2="28" stroke="#1e293b" stroke-width="2" stroke-linecap="round" transform="rotate(-90,70,72)" id="battery_needle" style="transition:transform .5s"/><circle cx="70" cy="72" r="3" fill="#1e293b"/><text x="18" y="76" font-size="7" fill="#ef4444">0</text><text x="66" y="16" font-size="7" fill="#10b981" text-anchor="middle">50</text><text x="118" y="76" font-size="7" fill="#0d9488">100</text></svg><div class="g-info"><div class="gv"><span id="battery_val">0</span></div><div class="gu">% · <span id="battery_voltage">--V</span> · <span id="battery_est" style="font-size:9px;color:var(--text3)">--</span></div></div></div></div>
  <div class="sc"><div class="sc-row"><div style="flex:1"><div class="sc-label">机械臂状态</div><div class="arm-r"><div class="arm-box"><canvas id="arm_canvas" width="140" height="70"></canvas></div><div class="arm-d"><div><span class="al">阶段</span><span class="av bad" id="arm_stage">未连接</span></div><div><span class="al">状态</span><span class="av bad" id="arm_state">离线</span></div><div><span class="al">活动量</span><span class="av" id="arm_motion">--</span></div></div></div><div class="bar-bg"><div class="bar-fg" id="arm_bar" style="width:0%"></div></div></div><div style="display:flex;flex-direction:column;gap:1px;align-items:center"><span style="font-size:18px;font-weight:800;color:var(--text3);line-height:1">6</span><span style="font-size:8px;color:var(--text3);font-weight:600">DOF</span></div></div></div>
</div>
<div style="display:flex;flex-direction:column;gap:8px;min-height:0">
  <div class="pnl c1" style="flex:1;min-height:0">
    <div class="pnl-h"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="1 6 1 22 8 18 16 22 23 18 23 2 16 6 8 2 1 6"/></svg>导航地图</div>
    <div class="pnl-b" style="padding:0 6px 4px">
      <div class="map-area" id="map_box"><canvas id="map_canvas"></canvas><div class="map-ph" id="map_empty"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><rect x="3" y="3" width="18" height="18" rx="2"/><path d="M3 9h18M9 3v18"/></svg><div class="mt">等待导航地图</div><div class="ms" id="map_state">/map</div></div></div>
      <div class="map-ft"><span class="map-badge">轨迹 <b id="track_count">0</b> 点</span><span class="map-st" id="map_ready">map 尚未就绪</span></div>
    </div>
  </div>
</div>
<div style="display:flex;flex-direction:column;gap:3px;min-height:0">
  <div class="pnl c3" style="flex:1;min-height:0"><div class="pnl-h"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="10" r="3"/><path d="M12 21.7C17.3 17 20 13 20 10a8 8 0 1 0-16 0c0 3 2.7 7 8 11.7z"/></svg>位置与航向<span class="si si-ok" style="margin-left:auto" id="pos_fix">已定位</span></div><div class="pnl-b" style="justify-content:center;gap:3px;padding:4px 7px 5px"><div style="display:flex;align-items:center;gap:6px"><div style="position:relative;width:54px;height:54px;flex-shrink:0"><svg viewBox="0 0 54 54" width="54" height="54" id="compass_svg"><defs><linearGradient id="cg1" x1="0" y1="0" x2="0" y2="1"><stop offset="0%" stop-color="#10b981" stop-opacity=".12"/><stop offset="100%" stop-color="transparent"/></linearGradient></defs><circle cx="27" cy="27" r="26" fill="url(#cg1)" stroke="#10b981" stroke-width=".5" stroke-opacity=".2"/><circle cx="27" cy="27" r="21" fill="none" stroke="#e2e8f0" stroke-width="1" stroke-dasharray="1.5 3.5"/><g stroke="#cbd5e1" stroke-width=".8"><line x1="27" y1="3" x2="27" y2="7"/><line x1="27" y1="47" x2="27" y2="51"/><line x1="3" y1="27" x2="7" y2="27"/><line x1="47" y1="27" x2="51" y2="27"/></g><text x="27" y="10" text-anchor="middle" font-size="5.5" fill="#10b981" font-weight="800">N</text><text x="27" y="51" text-anchor="middle" font-size="5" fill="#94a3b8" font-weight="600">S</text><text x="5" y="29" text-anchor="middle" font-size="5" fill="#94a3b8" font-weight="600">W</text><text x="49" y="29" text-anchor="middle" font-size="5" fill="#94a3b8" font-weight="600">E</text><g id="compass_needle" style="transition:transform .6s cubic-bezier(.4,0,.2,1);transform-origin:27px 27px"><polygon points="27,7 25,21 27,19 29,21" fill="#10b981"/><polygon points="27,47 25,33 27,35 29,33" fill="#94a3b8"/></g><circle cx="27" cy="27" r="3" fill="#1e293b"/><circle cx="27" cy="27" r="1.5" fill="#0d9488"/></svg></div><div style="flex:1;display:flex;flex-direction:column;gap:2px"><div style="display:flex;align-items:baseline;gap:3px"><span style="font-size:8px;color:#0d9488;font-weight:800;width:14px;text-align:center;background:rgba(13,148,136,.08);border-radius:3px;padding:1px 0">X</span><span style="font-size:15px;font-weight:800;color:var(--text);font-variant-numeric:tabular-nums" id="x">0.00</span><span style="font-size:8px;color:var(--text3)">m</span><div class="mini-bar" style="flex:1;height:3px;margin-left:4px"><div class="mini-bar-fg" id="x_bar" style="width:45%;background:linear-gradient(90deg,#0d9488,#14b8a6)"></div></div></div><div style="display:flex;align-items:baseline;gap:3px"><span style="font-size:8px;color:#10b981;font-weight:800;width:14px;text-align:center;background:rgba(16,185,129,.06);border-radius:3px;padding:1px 0">Y</span><span style="font-size:15px;font-weight:800;color:var(--text);font-variant-numeric:tabular-nums" id="y">0.00</span><span style="font-size:8px;color:var(--text3)">m</span><div class="mini-bar" style="flex:1;height:3px;margin-left:4px"><div class="mini-bar-fg" id="y_bar" style="width:30%;background:linear-gradient(90deg,#10b981,#06b6d4)"></div></div></div></div></div><div style="display:flex;gap:3px"><div style="flex:1;text-align:center;padding:3px 0;background:linear-gradient(135deg,rgba(13,148,136,.05),rgba(16,185,129,.04));border-radius:6px;border:1px solid rgba(13,148,136,.12)"><div style="font-size:8px;color:var(--accent);font-weight:700">YAW</div><div style="font-size:17px;font-weight:800;color:var(--accent);font-variant-numeric:tabular-nums;line-height:1.1" id="yaw">0.00</div><div style="font-size:8px;color:var(--text3)">rad</div></div><div style="flex:1;text-align:center;padding:3px 0;background:#f8fafc;border-radius:6px;border:1px solid var(--border2)"><div style="font-size:8px;color:var(--text3);font-weight:700">角度</div><div style="font-size:17px;font-weight:800;color:var(--text);font-variant-numeric:tabular-nums;line-height:1.1" id="yaw_deg">0.0</div><div style="font-size:8px;color:var(--text3)">deg</div></div><div style="flex:1;text-align:center;padding:3px 0;background:#f8fafc;border-radius:6px;border:1px solid var(--border2)"><div style="font-size:8px;color:var(--text3);font-weight:700">距离</div><div style="font-size:17px;font-weight:800;color:var(--text);font-variant-numeric:tabular-nums;line-height:1.1" id="pos_dist">0.00</div><div style="font-size:8px;color:var(--text3)">m</div></div></div></div></div>
  <div class="pnl c4" style="flex:1;min-height:0"><div class="pnl-h"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M5 12h14M12 5l7 7-7 7"/></svg>运动速度<span class="si si-ok" style="margin-left:auto;font-size:9px" id="motion_tag">静止</span></div><div class="pnl-b" style="justify-content:center;gap:3px;padding:4px 7px 5px"><div style="display:flex;gap:4px"><div style="flex:2;text-align:center;padding:4px 0;background:linear-gradient(180deg,rgba(16,185,129,.06),transparent);border-radius:6px;border:1px solid rgba(16,185,129,.1)"><div style="font-size:8px;color:var(--green);font-weight:700">前进 Vx</div><div style="font-size:20px;font-weight:800;color:var(--green);line-height:1.1;font-variant-numeric:tabular-nums" id="vx">0.00</div><div style="font-size:8px;color:var(--text3)">m/s</div><div class="mini-bar" style="height:4px;margin:2px 6px 0"><div class="mini-bar-fg" id="vx_bar" style="width:35%;background:linear-gradient(90deg,#10b981,#06b6d4)"></div></div></div><div style="flex:1;display:flex;flex-direction:column;gap:3px"><div style="flex:1;text-align:center;padding:2px 0;background:#f8fafc;border-radius:5px;border:1px solid var(--border2);display:flex;flex-direction:column;justify-content:center"><div style="font-size:7px;color:var(--text3);font-weight:700">Vy</div><div style="font-size:12px;font-weight:800;color:var(--text);font-variant-numeric:tabular-nums" id="vy">0.00</div><div class="mini-bar" style="margin:1px 4px 0"><div class="mini-bar-fg" id="vy_bar" style="width:15%;background:#06b6d4"></div></div></div><div style="flex:1;text-align:center;padding:2px 0;background:#f8fafc;border-radius:5px;border:1px solid var(--border2);display:flex;flex-direction:column;justify-content:center"><div style="font-size:7px;color:var(--text3);font-weight:700">Wz</div><div style="font-size:12px;font-weight:800;color:var(--text);font-variant-numeric:tabular-nums" id="wz">0.00</div><div class="mini-bar" style="margin:1px 4px 0"><div class="mini-bar-fg" id="wz_bar" style="width:22%;background:#14b8a6"></div></div></div></div></div><div style="display:flex;align-items:center;gap:5px;padding:3px 5px;background:#f8fafc;border-radius:5px;border:1px solid var(--border2)"><div style="width:30px;height:30px;position:relative;flex-shrink:0"><svg viewBox="0 0 30 30" width="30" height="30"><circle cx="15" cy="15" r="13" fill="none" stroke="#e2e8f0" stroke-width=".8"/><circle cx="15" cy="15" r="8" fill="none" stroke="#f1f5f9" stroke-width=".5" stroke-dasharray="1 2"/><line x1="15" y1="2" x2="15" y2="28" stroke="#f1f5f9" stroke-width=".5"/><line x1="2" y1="15" x2="28" y2="15" stroke="#f1f5f9" stroke-width=".5"/><line x1="15" y1="15" x2="15" y2="6" stroke="#10b981" stroke-width="2" stroke-linecap="round" id="vec_arrow" style="transition:all .5s"/><circle cx="15" cy="15" r="1.5" fill="#10b981"/></svg></div><div style="flex:1"><div style="display:flex;justify-content:space-between;align-items:baseline"><span style="font-size:8px;color:var(--text3);font-weight:600">合成速度 |v|</span><span style="font-size:13px;font-weight:800;color:var(--green);font-variant-numeric:tabular-nums" id="v_total">0.00<span style="font-size:8px;color:var(--text3);font-weight:500"> m/s</span></span></div><div class="mini-bar" style="height:3px;margin-top:2px"><div class="mini-bar-fg" id="v_total_bar" style="width:30%;background:linear-gradient(90deg,#10b981,#06b6d4,#0d9488)"></div></div></div></div></div></div>
  <div class="pnl c5" style="flex:1;min-height:0"><div class="pnl-h"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg>传感器状态<span class="si si-ok" style="margin-left:auto;font-size:9px">全部正常</span></div><div class="pnl-b" style="justify-content:center;gap:6px;padding:6px 9px 7px"><div class="dr" style="padding:7px 0"><span class="dl" style="font-size:12px"><svg viewBox="0 0 24 24" width="13" height="13" fill="none" stroke="#0d9488" stroke-width="2" style="margin-right:2px"><circle cx="12" cy="12" r="3"/><path d="M12 1v4M12 19v4M4.2 4.2l2.8 2.8M17 17l2.8 2.8M1 12h4M19 12h4M4.2 19.8l2.8-2.8M17 7l2.8-2.8"/></svg>IMU</span><span class="si si-ok" id="sensor_imu"><span class="dot"></span>正常</span></div><div class="dr" style="padding:7px 0"><span class="dl" style="font-size:12px"><svg viewBox="0 0 24 24" width="13" height="13" fill="none" stroke="#0d9488" stroke-width="2" style="margin-right:2px"><circle cx="12" cy="12" r="10"/><circle cx="12" cy="12" r="3"/></svg>雷达</span><span class="si si-ok" id="sensor_lidar"><span class="dot"></span>正常</span></div><div class="dr" style="padding:7px 0"><span class="dl" style="font-size:12px"><svg viewBox="0 0 24 24" width="13" height="13" fill="none" stroke="#0d9488" stroke-width="2" style="margin-right:2px"><rect x="2" y="6" width="20" height="14" rx="2"/><circle cx="12" cy="13" r="4"/></svg>深度相机</span><span class="si si-ok" id="sensor_camera"><span class="dot"></span>正常</span></div></div></div>
</div>
<div style="display:flex;flex-direction:column;gap:3px;min-height:0">
  <div class="pnl c3" style="flex-shrink:0"><div class="pnl-h"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M9 11l3 3L22 4"/><path d="M21 12v7a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11"/></svg>当前任务</div><div class="pnl-b" style="justify-content:flex-start;gap:2px;padding:4px 9px 6px"><div class="dr"><span class="dl">任务状态</span><span class="dv" id="task_state" style="color:var(--green);font-weight:700">待分配</span></div><div class="dr"><span class="dl">目标科室</span><span class="dv" id="task_target">护士站</span></div><div class="dr"><span class="dl">配送药品</span><span class="dv" id="task_medicine">暂无</span></div><div class="dr" style="border-bottom:none"><span class="dl">任务编号</span><span class="dv" id="task_id">--</span></div></div></div>
  <div class="pnl c2" style="flex-shrink:0"><div class="pnl-h"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M18 10h-1.26A8 8 0 1 0 9 20h9a5 5 0 0 0 0-10z"/></svg>云平台<span class="si si-warn" style="margin-left:auto" id="cloud_tag2">待连接</span><span class="tag te hidden" style="margin-left:auto;font-size:8px" id="cloud_tag">离线</span></div><div class="pnl-b" style="justify-content:flex-start;gap:3px;padding:4px 8px 5px"><div class="cloud-ep" id="cloud_endpoint">--</div><div class="cloud-stats"><div class="cloud-stat"><div class="cs-v" style="color:var(--green)" id="cloud_msg_count">--</div><div class="cs-l">消息</div></div><div class="cloud-stat"><div class="cs-v" style="color:var(--blue)" id="cloud_event_count">--</div><div class="cs-l">事件</div></div><div class="cloud-stat"><div class="cs-v" style="color:var(--accent)" id="cloud_latency">--</div><div class="cs-l">ms</div></div></div><div class="cloud-events" id="cloud_events_list"></div><div class="dr" style="padding:2px 0;border:none"><span class="dl">最近事件</span><span class="dv" id="cloud_event" style="font-size:11px">--</span></div></div></div>
  <div class="pnl c4" style="flex:1;min-height:0"><div class="pnl-h"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><polyline points="17 8 12 3 7 8"/><line x1="12" y1="3" x2="12" y2="15"/></svg>上传记录<span style="margin-left:auto;font-size:14px;font-weight:800" id="upload_count">0</span><span style="font-size:9px;color:var(--text3)">条</span></div><div class="pnl-b" style="padding:3px 6px 4px;gap:3px;justify-content:flex-start"><textarea class="cloud-log" id="cloud_upload_log" readonly spellcheck="false">等待上传数据</textarea></div></div>
</div>
</div>
<div class="bbar">
  <div class="cmds">
    <button class="cb command-btn" data-api="/api/start_base"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="2" y="6" width="20" height="12" rx="2"/><circle cx="8" cy="18" r="2"/><circle cx="16" cy="18" r="2"/></svg>底盘通信<small>base</small></button>
    <button class="cb command-btn" data-api="/api/start_lidar"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><path d="M12 2a10 10 0 0 1 0 20" fill="rgba(13,148,136,.1)"/><circle cx="12" cy="12" r="3"/></svg>雷达扫描<small>lidar</small></button>
    <button class="cb command-btn acc" data-api="/api/start_mapping"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="1 6 1 22 8 18 16 22 23 18 23 2 16 6 8 2 1 6"/><line x1="8" y1="2" x2="8" y2="18"/><line x1="16" y1="6" x2="16" y2="22"/></svg>开始建图<small>carto</small></button>
    <button class="cb command-btn" data-api="/api/save_map"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"/><polyline points="17 21 17 13 7 13 7 21"/></svg>保存地图<small>save</small></button>
    <button class="cb command-btn" data-api="/api/start_keyboard"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="2" y="4" width="20" height="16" rx="2"/><path d="M6 8h.01M10 8h.01M14 8h.01M18 8h.01M8 12h.01M12 12h.01M16 12h.01M8 16h8"/></svg>键盘遥控<small>key</small></button>
    <button class="cb command-btn acc" data-api="/api/start_nav"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="10" r="3"/><path d="M12 21.7C17.3 17 20 13 20 10a8 8 0 1 0-16 0c0 3 2.7 7 8 11.7z"/></svg>送药系统<small>voice</small></button>
  </div>
  <div class="bst" id="control_status">控制台就绪</div>
</div>
</div>
<!-- Hidden adapter elements for backend JS compatibility -->
<span id="speed_gauge" class="hidden"></span><span id="battery_gauge" class="hidden"></span>
<span id="speed_bar" class="hidden"></span><span id="battery_bar" class="hidden"></span>
<canvas id="speed_spark" class="hidden" width="1" height="1"></canvas>
<canvas id="battery_spark" class="hidden" width="1" height="1"></canvas>
<span id="speed_trend" class="hidden"></span><span id="battery_trend" class="hidden"></span>
<div class="snackbar-container" id="snackbar_container"></div>
  <script>
    const $=id=>document.getElementById(id);
    function set(id,v){const e=$(id);if(e)e.textContent=v}
    function n(v){return Number(v||0)}
    function clamp(v,lo,hi){return Math.max(lo,Math.min(hi,v))}
    function cls(id,c){const e=$(id);if(e)e.className=c}
    const snackQueue=[];let snackBusy=false;
    function snack(msg,action,dur){dur=dur||4000;const c=$('snackbar_container'),el=document.createElement('div');el.className='snackbar';const t=document.createElement('span');t.textContent=msg;el.appendChild(t);if(action){const b=document.createElement('button');b.className='snack-action';b.textContent=action.text;b.onclick=()=>{action.fn();dismiss()};el.appendChild(b)}c.appendChild(el);const dismiss=()=>{el.classList.add('out');setTimeout(()=>el.remove(),300)};setTimeout(dismiss,dur)}
    function addRipple(e){const btn=e.currentTarget,r=btn.getBoundingClientRect(),sz=Math.max(r.width,r.height),rp=document.createElement('span');rp.className='ripple';rp.style.width=rp.style.height=sz+'px';rp.style.left=(e.clientX-r.left-sz/2)+'px';rp.style.top=(e.clientY-r.top-sz/2)+'px';btn.appendChild(rp);setTimeout(()=>rp.remove(),650)}
    document.querySelectorAll('.btn,.cb').forEach(b=>b.addEventListener('click',addRipple));
    const canvas=$('map_canvas'),box=$('map_box'),ctx=canvas.getContext('2d');
    let map=null,mapImage=null,pose=null,track=[],cloudUploadSignature='';
    function resizeCanvas(){const r=box.getBoundingClientRect();canvas.width=Math.max(320,Math.floor(r.width*devicePixelRatio));canvas.height=Math.max(240,Math.floor(r.height*devicePixelRatio));drawMap()}
    window.addEventListener('resize',resizeCanvas);
    // 区域使用地图像素坐标，直接贴合 qing_slam_map.pgm 的墙体像素。
    const mapRegions=[
      {name:'主走廊',fill:'rgba(148,163,184,.12)',stroke:'rgba(100,116,139,.22)',label:{px:210,py:58},points:[{px:20,py:47},{px:20,py:31},{px:37,py:31},{px:37,py:16},{px:87,py:15},{px:137,py:49},{px:187,py:9},{px:197,py:10},{px:206,py:12},{px:219,py:13},{px:235,py:14},{px:249,py:16},{px:277,py:17},{px:291,py:19},{px:305,py:20},{px:309,py:22},{px:320,py:32},{px:328,py:41},{px:328,py:75},{px:306,py:98},{px:277,py:98},{px:262,py:98},{px:230,py:94},{px:210,py:93},{px:190,py:91},{px:178,py:89},{px:137,py:84},{px:122,py:76},{px:105,py:66},{px:87,py:50},{px:47,py:48}]},
      {name:'休息区',fill:'rgba(45,212,191,.18)',stroke:'rgba(13,148,136,.48)',label:{px:37,py:39},points:[{px:21,py:31},{px:53,py:31},{px:53,py:47},{px:20,py:47},{px:20,py:31}]},
      {name:'药房',fill:'rgba(251,191,36,.20)',stroke:'rgba(217,119,6,.44)',label:{px:89,py:31},points:[{px:78,py:16},{px:87,py:16},{px:105,py:27},{px:105,py:40},{px:95,py:47},{px:78,py:46},{px:78,py:31}]},
      {name:'101病房',fill:'rgba(96,165,250,.18)',stroke:'rgba(37,99,235,.42)',label:{px:211,py:24},points:[{px:187,py:9},{px:197,py:10},{px:206,py:12},{px:219,py:13},{px:235,py:14},{px:235,py:30},{px:187,py:30}]},
      {name:'102病房',fill:'rgba(74,222,128,.17)',stroke:'rgba(22,163,74,.42)',label:{px:205,py:84},points:[{px:178,py:71},{px:190,py:72},{px:208,py:74},{px:229,py:76},{px:229,py:94},{px:210,py:93},{px:190,py:91},{px:178,py:89}]},
      {name:'103病房',fill:'rgba(167,139,250,.18)',stroke:'rgba(124,58,237,.42)',label:{px:306,py:30},points:[{px:277,py:17},{px:291,py:19},{px:305,py:20},{px:309,py:22},{px:320,py:32},{px:328,py:41},{px:277,py:36}]},
      {name:'104病房',fill:'rgba(251,146,60,.19)',stroke:'rgba(234,88,12,.42)',label:{px:304,py:88},points:[{px:277,py:79},{px:296,py:78},{px:314,py:77},{px:328,py:75},{px:306,py:98},{px:277,py:98}]}
    ];
    function worldToMap(x,y){return{x:(x-map.origin.x)/map.resolution,y:map.height-(y-map.origin.y)/map.resolution}}
    function fitFor(){const rotate=(map.width>map.height)!=(canvas.width>canvas.height),w=rotate?map.height:map.width,h=rotate?map.width:map.height,s=Math.min(canvas.width/w,canvas.height/h)*0.94;return{scale:s,rotate,ox:(canvas.width-w*s)/2,oy:(canvas.height-h*s)/2}}
    function mapToCanvas(q,f){return f.rotate?{x:f.ox+(map.height-q.y)*f.scale,y:f.oy+q.x*f.scale}:{x:f.ox+q.x*f.scale,y:f.oy+q.y*f.scale}}
    function regionPoint(p,f){return p.px!==undefined?mapToCanvas({x:p.px,y:p.py},f):mapToCanvas(worldToMap(p.x,p.y),f)}
    function drawRegionLabel(r,f){const p=regionPoint(r.label,f),fs=Math.max(11,Math.min(18,Math.round(12*devicePixelRatio)));ctx.font=`700 ${fs}px 'Microsoft YaHei',system-ui,sans-serif`;ctx.textAlign='center';ctx.textBaseline='middle';const padX=7*devicePixelRatio,padY=4*devicePixelRatio,tw=ctx.measureText(r.name).width,rx=p.x-tw/2-padX,ry=p.y-fs/2-padY;ctx.fillStyle='rgba(255,255,255,.82)';ctx.strokeStyle='rgba(15,23,42,.16)';ctx.lineWidth=Math.max(1,devicePixelRatio);ctx.beginPath();ctx.roundRect(rx,ry,tw+padX*2,fs+padY*2,6*devicePixelRatio);ctx.fill();ctx.stroke();ctx.fillStyle='rgba(30,41,59,.92)';ctx.fillText(r.name,p.x,p.y)}
    function drawRegions(f){ctx.save();ctx.globalCompositeOperation='multiply';mapRegions.forEach(r=>{ctx.beginPath();r.points.forEach((pt,i)=>{const p=regionPoint(pt,f);if(i===0)ctx.moveTo(p.x,p.y);else ctx.lineTo(p.x,p.y)});ctx.closePath();ctx.fillStyle=r.fill;ctx.strokeStyle=r.stroke;ctx.lineWidth=Math.max(1,1.4*devicePixelRatio);ctx.fill();ctx.stroke()});ctx.restore();mapRegions.forEach(r=>drawRegionLabel(r,f))}
    function drawMap(){ctx.clearRect(0,0,canvas.width,canvas.height);ctx.fillStyle='#e8ecf1';ctx.fillRect(0,0,canvas.width,canvas.height);if(!map||!mapImage){$('map_empty').style.display='flex';return}$('map_empty').style.display='none';const f=fitFor();ctx.imageSmoothingEnabled=false;ctx.save();if(f.rotate){ctx.translate(f.ox+map.height*f.scale,f.oy);ctx.rotate(Math.PI/2);ctx.drawImage(mapImage,0,0,map.width*f.scale,map.height*f.scale)}else{ctx.drawImage(mapImage,f.ox,f.oy,map.width*f.scale,map.height*f.scale)}ctx.restore();drawRegions(f);if(track.length>1){ctx.strokeStyle='#0d9488';ctx.lineWidth=Math.max(2,3*devicePixelRatio);ctx.beginPath();track.forEach((p,i)=>{const c=mapToCanvas(worldToMap(p.x,p.y),f);if(i===0)ctx.moveTo(c.x,c.y);else ctx.lineTo(c.x,c.y)});ctx.stroke()}if(pose){const c=mapToCanvas(worldToMap(pose.x,pose.y),f);ctx.save();ctx.translate(c.x,c.y);ctx.rotate(f.rotate?Math.PI/2-pose.yaw:-pose.yaw);ctx.fillStyle='#0d9488';ctx.strokeStyle='#fff';ctx.lineWidth=2*devicePixelRatio;ctx.beginPath();ctx.moveTo(13*devicePixelRatio,0);ctx.lineTo(-9*devicePixelRatio,-8*devicePixelRatio);ctx.lineTo(-4*devicePixelRatio,0);ctx.lineTo(-9*devicePixelRatio,8*devicePixelRatio);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore()}}
    async function loadMap(){try{const r=await fetch('/api/map',{cache:'no-store'});const m=await r.json();if(!m.ready){set('map_state','/map 尚未就绪');return}map=m;const off=document.createElement('canvas');off.width=m.width;off.height=m.height;const c=off.getContext('2d'),img=c.createImageData(m.width,m.height);for(let y=0;y<m.height;y++){for(let x=0;x<m.width;x++){const src=(m.height-1-y)*m.width+x,val=m.data[src],i=(y*m.width+x)*4;let g=200;if(val<0)g=230;else if(val===0)g=255;else g=Math.max(100,240-val*2);img.data[i]=g;img.data[i+1]=g;img.data[i+2]=g;img.data[i+3]=255}}c.putImageData(img,0,0);mapImage=off;set('map_state',`地图 ${m.width}×${m.height}`);resizeCanvas()}catch(e){set('map_state','地图接口读取失败')}}
    const speedHist=[],batteryHist=[],HIST_MAX=60;
    function drawSparkline(){}
    function updateTrend(){}
    const BATTERY_EMPTY_V=10.0,BATTERY_FULL_V=12.5;
    let failCount=0;
    function armPhaseProgress(p){return{idle:8,observe:28,pick_down:56,pick_up:82,place:100}[p]||0}
    function armPhaseBusy(p){return p==='pick_down'||p==='pick_up'||p==='place'}
    async function tick(){try{const r=await fetch('/api/status',{cache:'no-store'});const d=await r.json(),speed=Math.hypot(n(d.velocity.linear_x),n(d.velocity.linear_y)),voltage=n(d.power.voltage),battery=clamp((voltage-BATTERY_EMPTY_V)/(BATTERY_FULL_V-BATTERY_EMPTY_V),0,1),fresh=Math.max(n(d.age.odom),n(d.age.voltage),n(d.age.imu));failCount=0;set('system_state','在线');cls('system_state','badge badge-on');set('speed_gauge',speed.toFixed(2));set('speed_val',speed.toFixed(2));set('battery_gauge',Math.round(battery*100));set('battery_val',Math.round(battery*100));set('battery_voltage',voltage>0?voltage.toFixed(2)+'V':'--V');const isMoving=speed>0.03||Math.abs(n(d.velocity.angular_z))>0.05;set('motion_state',isMoving?'运行中':'待机');set('x',d.pose.x+' m');set('y',d.pose.y+' m');set('yaw',d.pose.yaw+' rad');set('vx',d.velocity.linear_x+' m/s');set('vy',d.velocity.linear_y+' m/s');set('wz',d.velocity.angular_z+' rad/s');set('charging',d.power.charging?'充电中':'未充电');set('current',d.power.charging_current);set('red',d.power.red_flag);set('imu_wz',d.imu.angular_z+' rad/s');set('imu_ax',d.imu.accel_x+' m/s2');set('imu_ay',d.imu.accel_y+' m/s2');const arm=d.arm||{},armPhase=arm.phase||'unknown',armPhaseOnline=armPhase!=='unknown'&&n(arm.phase_age)<5,armOk=n(d.age.arm)<5||armPhaseOnline,armMoving=!!arm.moving||armPhaseBusy(armPhase);set('arm_stage',!armOk?'未连接':armPhaseOnline?(arm.phase_label||'未知阶段'):(armMoving?'运行中':'稳定待命'));set('arm_state',!armOk?'离线':armMoving?'动作中':armPhase==='idle'?'待机':'稳定');cls('arm_stage',!armOk?'av bad':'av ok');cls('arm_state',!armOk?'av bad':armMoving?'av ok':'av ok');set('arm_motion',arm.motion!==undefined?arm.motion:'--');const armBar=$('arm_bar');if(armBar)armBar.style.width=armPhaseProgress(armPhase)+'%';const task=d.task||{};set('task_state',task.state||'待分配');set('task_target',task.target||task.target_room||'护士站');set('task_medicine',task.medicine||'暂无');set('task_id',task.task_id||'--');const taskStateEl=$('task_state');if(taskStateEl)taskStateEl.style.color=task.active?'var(--green)':task.state==='异常'?'var(--red)':'var(--text2)';set('fresh_state',fresh<2?'正常':fresh<5?'延迟':'过期');pose={x:n(d.pose.x),y:n(d.pose.y),yaw:n(d.pose.yaw)};if(track.length===0||Math.hypot(track[track.length-1].x-pose.x,track[track.length-1].y-pose.y)>0.02){track.push({...pose});if(track.length>900)track.shift();set('track_count',track.length+' 点')}drawMap();speedHist.push(speed);batteryHist.push(battery*100);if(speedHist.length>HIST_MAX)speedHist.shift();if(batteryHist.length>HIST_MAX)batteryHist.shift()}catch(e){failCount++;if(failCount===1)snack('与机器人连接中断，正在重试...');if(failCount%5===0&&failCount>1)snack('连接失败 '+failCount+' 次，持续重试中');set('system_state','连接异常');cls('system_state','badge badge-off')}}
    function compactLog(v){return String(v||'').replace(/^.*payload=/,'').replace(/^\[[^\]]+\]\s*/,'').replace(/\s+/g,' ').trim()}
    function uploadRecord(v){const p=compactLog(v);try{const o=JSON.parse(p||'{}'),m=o.params&&o.params.medicine_delivery;if(m)return{data:m.value||{},time:m.time||0,raw:p};return{data:o,time:o.timestamp_ms||o.time||0,raw:p}}catch(e){return{data:{status:'待同步'},time:0,raw:p}}}
    function uploadKey(r){const d=r.data||{},key=[d.task_id||'',d.medicine||'',d.room||d.target_room||'',d.status||d.state||''].join('|');return key==='|||'?(r.raw||key):key}
    function uploadBlock(r,i){const d=r.data||{},t=r.time||d.timestamp_ms||0,ts=t?new Date(Number(t)).toLocaleTimeString('zh-CN',{hour12:false}):'--:--:--';return `${String(i+1).padStart(2,'0')}  ${ts}  药品 ${d.medicine||'--'} · 房间 ${d.room||d.target_room||'--'} · ${d.status||d.state||'待同步'}`}
    function renderUploadLog(items){const log=$('cloud_upload_log'),seen=new Set(),records=[];(items||[]).forEach(v=>{const r=uploadRecord(v),k=uploadKey(r);if(seen.has(k))return;seen.add(k);records.push(r)});const text=records.length?records.map(uploadBlock).join('\n'):'暂无上传数据';set('upload_count',records.length);if(text!==cloudUploadSignature){log.value=text;cloudUploadSignature=text}}
    async function loadCloud(){try{const r=await fetch('/api/cloud',{cache:'no-store'});const d=await r.json(),state=d.online?'online':d.state==='connecting'?'connecting':'offline';const label=d.online?'已连接':state==='connecting'?'连接中':'未连接';set('cloud_state',d.online?'云平台在线':state==='connecting'?'连接中':'离线');cls('cloud_state','badge '+(d.online?'badge-on':'badge-off'));set('cloud_endpoint',d.endpoint||'--');set('cloud_tag',d.online?'在线':state==='connecting'?'连接中':'离线');set('cloud_tag2',label);cls('cloud_tag2','si '+(d.online?'si-ok':state==='connecting'?'si-warn':'si-err'));renderUploadLog(Array.isArray(d.uploads)?d.uploads:(d.last_upload?[d.last_upload]:[]))}catch(e){set('cloud_state','云平台异常');cls('cloud_state','badge badge-off');set('cloud_tag2','异常');cls('cloud_tag2','si si-err');$('cloud_upload_log').value='无法读取云平台状态'}}
    /* Gauge adapter: sync SVG arcs when hidden speed_gauge/battery_gauge are updated */
    function updateGauge(arcId,needleId,valElId,maxVal){const valEl=$(valElId),arc=$(arcId),needle=$(needleId);if(!valEl||!arc)return;const pct=clamp(parseFloat(valEl.textContent||0)/maxVal,0,1);const total=173;arc.setAttribute('stroke-dashoffset',total-(total*pct));needle.setAttribute('transform',`rotate(${-90+180*pct},70,72)`)}
    new MutationObserver(()=>{updateGauge('speed_arc','speed_needle','speed_gauge',0.8);updateGauge('battery_arc','battery_needle','battery_gauge',100)}).observe($('speed_gauge'),{childList:true,characterData:true,subtree:true});
    new MutationObserver(()=>updateGauge('battery_arc','battery_needle','battery_gauge',100)).observe($('battery_gauge'),{childList:true,characterData:true,subtree:true});
    /* Cloud tag sync */
    new MutationObserver(m=>{m.forEach(r=>{const t=r.target,txt=t.textContent||'';const tag=$('cloud_tag');if(tag)tag.textContent=txt;if(txt.includes('在线')){t.className='badge badge-on';t.classList.remove('badge-off')}else{t.className='badge badge-off';t.classList.remove('badge-on')}})}).observe($('cloud_state'),{childList:true,characterData:true,subtree:true});
    $('start_nav').addEventListener('click',async()=>{const b=$('start_nav');b.disabled=true;b.classList.add('loading');b.textContent='启动中';try{const r=await fetch('/api/start_nav',{method:'POST'}),d=await r.json();b.textContent=d.message||'已发送';snack('已发送送药系统启动命令')}catch(e){b.textContent='启动失败';snack('送药系统启动命令发送失败')}setTimeout(()=>{b.disabled=false;b.classList.remove('loading');b.textContent='启动送药系统'},3000)});
    document.querySelectorAll('.command-btn').forEach(b=>b.addEventListener('click',async()=>{b.disabled=true;b.classList.add('loading');const label=b.childNodes[0].textContent;set('control_status','正在执行: '+label);try{const r=await fetch(b.dataset.api,{method:'POST'}),d=await r.json();set('control_status',d.message||'命令已发送');snack('已发送: '+label)}catch(e){set('control_status','命令发送失败');snack('命令发送失败: '+label)}setTimeout(()=>{b.disabled=false;b.classList.remove('loading')},1600)}));
    resizeCanvas();loadMap();tick();loadCloud();
    setInterval(tick,500);setInterval(loadMap,10000);setInterval(loadCloud,1000);
    /* Clock */
    function updateClock(){const d=new Date(),pad=n=>String(n).padStart(2,'0');set('hdr_time',pad(d.getHours())+':'+pad(d.getMinutes())+':'+pad(d.getSeconds()));const days=['周日','周一','周二','周三','周四','周五','周六'];set('hdr_date',(d.getMonth()+1)+'/'+d.getDate()+' '+days[d.getDay()])}
    updateClock();setInterval(updateClock,1000);
    /* IMU wave + compass + bars */
    const imuHist={ax:[],ay:[],az:[]};const HLEN=60;
    function animIMU(){const ax=parseFloat($('imu_ax')?.textContent||0),ay=parseFloat($('imu_ay')?.textContent||0),az=parseFloat($('imu_az')?.textContent||0);
      const axn=Math.min(Math.abs(ax)/2,1),ayn=Math.min(Math.abs(ay)/2,1),azn=Math.min(Math.abs(az)/2,1);
      const axb=$('imu_ax_bar2'),ayb=$('imu_ay_bar2'),azb=$('imu_az_bar2');
      if(axb)axb.style.height=(axn*80+10)+'%';if(ayb)ayb.style.height=(ayn*80+10)+'%';if(azb)azb.style.height=(azn*80+10)+'%';
      const total=Math.sqrt(ax*ax+ay*ay+az*az);const tb=$('imu_total_bar');if(tb)tb.style.width=Math.min(total/10*100,100)+'%';
      set('imu_total',total.toFixed(1));
      imuHist.ax.push(axn);imuHist.ay.push(ayn);imuHist.az.push(azn);
      if(imuHist.ax.length>HLEN){imuHist.ax.shift();imuHist.ay.shift();imuHist.az.shift()}
      drawWave();requestAnimationFrame(animIMU)}
    function drawWave(){const cv=$('imu_wave');if(!cv)return;const ctx=cv.getContext('2d');
      const w=cv.parentElement.clientWidth*2,h=cv.parentElement.clientHeight*2;if(cv.width!==w||cv.height!==h){cv.width=w;cv.height=h}
      ctx.clearRect(0,0,w,h);const draw=(arr,color)=>{if(arr.length<2)return;ctx.strokeStyle=color;ctx.lineWidth=1.5;ctx.beginPath();
      arr.forEach((v,i)=>{const x=i/(HLEN-1)*w,y=h-v*h*.8-h*.1;i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke()};
      draw(imuHist.ax,'rgba(13,148,136,.6)');draw(imuHist.ay,'rgba(6,182,212,.6)');draw(imuHist.az,'rgba(16,185,129,.6)')}
    /* IMU 实时波形面板已替换为传感器状态面板,动画循环不再启动 */
    /* Compass rotation from yaw */
    function updateCompass(){const yawEl=$('yaw');if(!yawEl)return;const yawRad=parseFloat(yawEl.textContent||0);const yawDeg=yawRad*180/Math.PI;
      const needle=$('compass_needle');if(needle)needle.style.transform='rotate('+yawDeg+'deg)';
      set('yaw_deg',yawDeg.toFixed(1));
      const xEl=$('x'),yEl=$('y');const xv=parseFloat(xEl?xEl.textContent:0),yv=parseFloat(yEl?yEl.textContent:0);
      set('pos_dist',Math.sqrt(xv*xv+yv*yv).toFixed(2))}
    setInterval(updateCompass,500);
    /* Sparkline animation */
    function animSpark(){const bars=document.querySelectorAll('.sc-spark span');bars.forEach(b=>{b.style.height=(20+Math.random()*80)+'%'});setTimeout(animSpark,2000)}
    setTimeout(animSpark,1000);
    /* Battery estimate */
    function updateBatEst(){const v=$('battery_val');if(!v)return;const pct=parseInt(v.textContent||0);const hours=(pct/100*5.4).toFixed(1);set('battery_est','~'+hours+'h')}
    setInterval(updateBatEst,3000);updateBatEst();
    /* Speed total */
    function updateSpeedTotal(){const vxEl=$('vx'),vyEl=$('vy');if(!vxEl)return;const vx=parseFloat(vxEl.textContent||0),vy=parseFloat(vyEl?vyEl.textContent:0);
      const total=Math.sqrt(vx*vx+vy*vy);set('v_total',total.toFixed(2));
      const bar=$('v_total_bar');if(bar)bar.style.width=Math.min(total/2*100,100)+'%';
      const tag=$('motion_tag');if(tag){tag.textContent=total>0.03?'运动中':'静止';tag.className=total>0.03?'si si-info':'si si-ok'}}
    setInterval(updateSpeedTotal,500);
    /* Arm canvas */
    const armCv=$('arm_canvas');if(armCv){const armCtx=armCv.getContext('2d');function drawArm(){
      const w=armCv.width,h=armCv.height;armCtx.clearRect(0,0,w,h);const cx=w*.35,cy=h*.72;
      armCtx.fillStyle='#cbd5e1';armCtx.fillRect(cx-12,cy-3,24,6);
      armCtx.strokeStyle='#64748b';armCtx.lineWidth=3;armCtx.lineCap='round';
      armCtx.beginPath();armCtx.moveTo(cx,cy-3);armCtx.lineTo(cx+8,cy-32);armCtx.stroke();
      armCtx.beginPath();armCtx.moveTo(cx+8,cy-32);armCtx.lineTo(cx+36,cy-44);armCtx.stroke();
      armCtx.beginPath();armCtx.moveTo(cx+36,cy-44);armCtx.lineTo(cx+60,cy-36);armCtx.stroke();
      [[cx,cy-3],[cx+8,cy-32],[cx+36,cy-44]].forEach(([jx,jy])=>{armCtx.fillStyle='#0d9488';armCtx.beginPath();armCtx.arc(jx,jy,4,0,Math.PI*2);armCtx.fill()});
      armCtx.strokeStyle='#94a3b8';armCtx.lineWidth=1.5;
      armCtx.beginPath();armCtx.moveTo(cx+60,cy-36);armCtx.lineTo(cx+68,cy-42);armCtx.stroke();
      armCtx.beginPath();armCtx.moveTo(cx+60,cy-36);armCtx.lineTo(cx+68,cy-30);armCtx.stroke();
      armCtx.fillStyle='#e2e8f0';armCtx.strokeStyle='#94a3b8';armCtx.lineWidth=1;
      const bx=cx-24,by=cy+3;armCtx.beginPath();armCtx.roundRect(bx,by,48,16,3);armCtx.fill();armCtx.stroke();
      armCtx.fillStyle='#475569';[bx+8,bx+40].forEach(wx=>{armCtx.beginPath();armCtx.arc(wx,by+18,5,0,Math.PI*2);armCtx.fill();armCtx.fillStyle='#94a3b8';armCtx.beginPath();armCtx.arc(wx,by+18,2,0,Math.PI*2);armCtx.fill();armCtx.fillStyle='#475569'})}
    drawArm()}
  </script>
</body>
</html>)HTML";

  }

  void send_response(int fd, const std::string &status, const std::string &content_type, const std::string &body)
  {
    std::ostringstream header;
    header << "HTTP/1.1 " << status << "\r\n"
           << "Content-Type: " << content_type << "\r\n"
           << "Content-Length: " << body.size() << "\r\n"
           << "Connection: close\r\n\r\n";
    const std::string response = header.str() + body;
    (void)::send(fd, response.data(), response.size(), MSG_NOSIGNAL);
  }

  void handle_client(int client_fd)
  {
    char buffer[1024] = {0};
    const ssize_t n = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
    if (n <= 0) return;
    const std::string req(buffer, static_cast<size_t>(n));
    if (req.rfind("GET /api/status", 0) == 0) {
      send_response(client_fd, "200 OK", "application/json", json_status());
    } else if (req.rfind("GET /api/cloud", 0) == 0) {
      send_response(client_fd, "200 OK", "application/json", json_cloud());
    } else if (req.rfind("GET /api/map", 0) == 0) {
      send_response(client_fd, "200 OK", "application/json", json_map());
    } else if (req.rfind("POST /api/start_nav", 0) == 0 || req.rfind("GET /api/start_nav", 0) == 0) {
      send_response(client_fd, "200 OK", "application/json", start_navigation());
    } else if (req.rfind("POST /api/start_base", 0) == 0 || req.rfind("GET /api/start_base", 0) == 0) {
      send_response(client_fd, "200 OK", "application/json", start_base());
    } else if (req.rfind("POST /api/start_lidar", 0) == 0 || req.rfind("GET /api/start_lidar", 0) == 0) {
      send_response(client_fd, "200 OK", "application/json", start_lidar());
    } else if (req.rfind("POST /api/start_mapping", 0) == 0 || req.rfind("GET /api/start_mapping", 0) == 0) {
      send_response(client_fd, "200 OK", "application/json", start_mapping());
    } else if (req.rfind("POST /api/save_map", 0) == 0 || req.rfind("GET /api/save_map", 0) == 0) {
      send_response(client_fd, "200 OK", "application/json", save_map());
    } else if (req.rfind("POST /api/start_keyboard", 0) == 0 || req.rfind("GET /api/start_keyboard", 0) == 0) {
      send_response(client_fd, "200 OK", "application/json", start_keyboard_control());
    } else if (req.rfind("GET / ", 0) == 0 || req.rfind("GET /index.html", 0) == 0) {
      send_response(client_fd, "200 OK", "text/html; charset=utf-8", html_page());
    } else {
      send_response(client_fd, "404 Not Found", "text/plain; charset=utf-8", "not found\n");
    }
  }

  void serve()
  {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "failed to create http socket");
      return;
    }
    const int fd_flags = fcntl(server_fd_, F_GETFD);
    if (fd_flags >= 0) {
      (void)fcntl(server_fd_, F_SETFD, fd_flags | FD_CLOEXEC);
    }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) {
      addr.sin_addr.s_addr = INADDR_ANY;
    }

    if (bind(server_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "failed to bind http server on %s:%d", host_.c_str(), port_);
      return;
    }
    if (listen(server_fd_, 16) < 0) {
      RCLCPP_ERROR(get_logger(), "failed to listen http server");
      return;
    }

    while (running_) {
      sockaddr_in client_addr{};
      socklen_t client_len = sizeof(client_addr);
      const int client_fd = accept(server_fd_, reinterpret_cast<sockaddr *>(&client_addr), &client_len);
      if (client_fd < 0) {
        if (running_) RCLCPP_WARN(get_logger(), "http accept failed");
        continue;
      }
      handle_client(client_fd);
      close(client_fd);
    }
  }

  std::string host_;
  int port_ = 8080;
  int server_fd_ = -1;
  std::atomic<bool> running_{true};
  std::thread server_thread_;
  std::mutex mutex_;
  CarStatus status_;
  MapStatus map_;
  std::vector<std::string> delivery_uploads_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr map_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr voltage_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<qing_robot_msgs::msg::UltrasonicArray>::SharedPtr ultrasonic_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr charging_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr charging_current_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr red_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_phase_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr delivery_record_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarWebNode>());
  rclcpp::shutdown();
  return 0;
}
