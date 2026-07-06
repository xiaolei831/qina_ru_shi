#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class LidarLoc : public rclcpp::Node
{
public:
  LidarLoc()
  : Node("lidar_loc"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    global_frame_ = this->declare_parameter<std::string>("global_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom_combined");
    laser_frame_ = this->declare_parameter<std::string>("laser_frame", "laser");
    map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
    laser_topic_ = this->declare_parameter<std::string>("laser_topic", "/scan");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
    initial_pose_topic_ = this->declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    broadcast_odom_tf_ = this->declare_parameter<bool>("broadcast_odom_tf", true);
    clear_costmaps_on_initial_pose_ =
      this->declare_parameter<bool>("clear_costmaps_on_initial_pose", true);
    clear_countdown_scans_ = this->declare_parameter<int>("clear_countdown_scans", 30);
    clear_service_names_ = this->declare_parameter<std::vector<std::string>>(
      "clear_services",
      {
        "/global_costmap/clear_entirely_global_costmap",
        "/local_costmap/clear_entirely_local_costmap",
      });

    for (const auto & service_name : clear_service_names_) {
      clear_costmap_clients_.push_back(
        this->create_client<nav2_msgs::srv::ClearEntireCostmap>(service_name));
    }

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos, std::bind(&LidarLoc::mapCallback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      laser_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarLoc::scanCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(30),
      std::bind(&LidarLoc::odomCallback, this, std::placeholders::_1));
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, rclcpp::QoS(1),
      std::bind(&LidarLoc::initialPoseCallback, this, std::placeholders::_1));

    pose_tf_timer_ = this->create_wall_timer(33ms, std::bind(&LidarLoc::poseTf, this));
  }

private:
  nav_msgs::msg::OccupancyGrid map_msg_;
  cv::Mat map_cropped_;
  cv::Mat map_temp_;
  sensor_msgs::msg::RegionOfInterest map_roi_info_;
  std::vector<cv::Point2f> scan_points_;
  std::deque<std::tuple<float, float, float>> data_queue_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::TimerBase::SharedPtr pose_tf_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr> clear_costmap_clients_;

  std::string global_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string laser_frame_;
  std::string map_topic_;
  std::string laser_topic_;
  std::string odom_topic_;
  std::string initial_pose_topic_;
  std::vector<std::string> clear_service_names_;
  bool broadcast_odom_tf_{true};
  bool clear_costmaps_on_initial_pose_{true};
  int clear_countdown_scans_{30};

  float lidar_x_{250.0f};
  float lidar_y_{250.0f};
  float lidar_yaw_{0.0f};
  const float deg_to_rad_{static_cast<float>(M_PI / 180.0)};
  int clear_countdown_{-1};
  int scan_count_{0};
  bool has_odom_transform_{false};
  geometry_msgs::msg::TransformStamped latest_odom_to_base_;

  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    setInitialPose(*msg);
  }

  void setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    const double map_x = msg.pose.pose.position.x;
    const double map_y = msg.pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg.pose.pose.orientation, q);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    if (map_msg_.info.resolution <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Map info is invalid or has not been received");
      return;
    }

    lidar_x_ = static_cast<float>(
      (map_x - map_msg_.info.origin.position.x) / map_msg_.info.resolution -
      map_roi_info_.x_offset);
    lidar_y_ = static_cast<float>(
      (map_y - map_msg_.info.origin.position.y) / map_msg_.info.resolution -
      map_roi_info_.y_offset);
    lidar_yaw_ = static_cast<float>(-yaw);

    clear_countdown_ = clear_countdown_scans_;
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_msg_ = *msg;
    cropMap();
    processMap();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_to_base_.header.stamp = msg->header.stamp;
    latest_odom_to_base_.header.frame_id = odom_frame_;
    latest_odom_to_base_.child_frame_id = base_frame_;
    latest_odom_to_base_.transform.translation.x = msg->pose.pose.position.x;
    latest_odom_to_base_.transform.translation.y = msg->pose.pose.position.y;
    latest_odom_to_base_.transform.translation.z = msg->pose.pose.position.z;
    latest_odom_to_base_.transform.rotation = msg->pose.pose.orientation;
    has_odom_transform_ = true;

    if (broadcast_odom_tf_) {
      tf_broadcaster_->sendTransform(latest_odom_to_base_);
    }
  }

  void cropMap()
  {
    const auto & info = map_msg_.info;
    int x_max = static_cast<int>(info.width / 2);
    int x_min = x_max;
    int y_max = static_cast<int>(info.height / 2);
    int y_min = y_max;
    bool first_point = true;

    cv::Mat map_raw(info.height, info.width, CV_8UC1, cv::Scalar(128));

    for (int y = 0; y < static_cast<int>(info.height); y++) {
      for (int x = 0; x < static_cast<int>(info.width); x++) {
        const int index = y * static_cast<int>(info.width) + x;
        map_raw.at<uchar>(y, x) = static_cast<uchar>(map_msg_.data[index]);

        if (map_msg_.data[index] == 100) {
          if (first_point) {
            x_max = x_min = x;
            y_max = y_min = y;
            first_point = false;
            continue;
          }
          x_min = std::min(x_min, x);
          x_max = std::max(x_max, x);
          y_min = std::min(y_min, y);
          y_max = std::max(y_max, y);
        }
      }
    }

    const int cen_x = (x_min + x_max) / 2;
    const int cen_y = (y_min + y_max) / 2;

    const int new_half_width = std::abs(x_max - x_min) / 2 + 50;
    const int new_half_height = std::abs(y_max - y_min) / 2 + 50;
    int new_origin_x = cen_x - new_half_width;
    int new_origin_y = cen_y - new_half_height;
    int new_width = new_half_width * 2;
    int new_height = new_half_height * 2;

    if (new_origin_x < 0) {
      new_origin_x = 0;
    }
    if ((new_origin_x + new_width) > static_cast<int>(info.width)) {
      new_width = static_cast<int>(info.width) - new_origin_x;
    }
    if (new_origin_y < 0) {
      new_origin_y = 0;
    }
    if ((new_origin_y + new_height) > static_cast<int>(info.height)) {
      new_height = static_cast<int>(info.height) - new_origin_y;
    }

    cv::Rect roi(new_origin_x, new_origin_y, new_width, new_height);
    map_cropped_ = map_raw(roi).clone();

    map_roi_info_.x_offset = new_origin_x;
    map_roi_info_.y_offset = new_origin_y;
    map_roi_info_.width = new_width;
    map_roi_info_.height = new_height;

    geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
    init_pose.pose.pose.position.x = 0.0;
    init_pose.pose.pose.position.y = 0.0;
    init_pose.pose.pose.position.z = 0.0;
    init_pose.pose.pose.orientation.x = 0.0;
    init_pose.pose.pose.orientation.y = 0.0;
    init_pose.pose.pose.orientation.z = 0.0;
    init_pose.pose.pose.orientation.w = 1.0;
    setInitialPose(init_pose);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (map_msg_.info.resolution <= 0.0 || map_cropped_.empty()) {
      return;
    }

    scan_points_.clear();
    double angle = msg->angle_min;

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        base_frame_, laser_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", ex.what());
      return;
    }

    tf2::Quaternion q_lidar;
    tf2::fromMsg(transform_stamped.transform.rotation, q_lidar);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q_lidar).getRPY(roll, pitch, yaw);

    const double tolerance = 0.1;
    bool lidar_is_inverted = std::abs(std::abs(roll) - M_PI) < tolerance;
    lidar_is_inverted = lidar_is_inverted && !(std::abs(std::abs(pitch) - M_PI) < tolerance);

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max) {
        const float x_laser = msg->ranges[i] * std::cos(angle);
        const float y_laser = -msg->ranges[i] * std::sin(angle);

        geometry_msgs::msg::PointStamped point_laser;
        point_laser.header.frame_id = laser_frame_;
        point_laser.header.stamp = msg->header.stamp;
        point_laser.point.x = x_laser;
        point_laser.point.y = y_laser;
        point_laser.point.z = 0.0;

        geometry_msgs::msg::PointStamped point_base;
        tf2::doTransform(point_laser, point_base, transform_stamped);

        float x = static_cast<float>(point_base.point.x / map_msg_.info.resolution);
        float y = static_cast<float>(point_base.point.y / map_msg_.info.resolution);
        if (lidar_is_inverted) {
          x = -x;
          y = -y;
        }

        scan_points_.push_back(cv::Point2f(x, y));
      }
      angle += msg->angle_increment;
    }

    if (scan_count_ == 0) {
      scan_count_++;
    }

    runScanMatch();

    if (clear_countdown_ > -1) {
      clear_countdown_--;
    }
    if (clear_countdown_ == 0 && clear_costmaps_on_initial_pose_) {
      clearCostmaps();
    }
  }

  void runScanMatch()
  {
    while (rclcpp::ok()) {
      if (!map_cropped_.empty()) {
        std::vector<cv::Point2f> transform_points;
        std::vector<cv::Point2f> clockwise_points;
        std::vector<cv::Point2f> counter_points;

        int max_sum = 0;
        float best_dx = 0.0f;
        float best_dy = 0.0f;
        float best_dyaw = 0.0f;

        for (const auto & point : scan_points_) {
          float rotated_x = point.x * std::cos(lidar_yaw_) - point.y * std::sin(lidar_yaw_);
          float rotated_y = point.x * std::sin(lidar_yaw_) + point.y * std::cos(lidar_yaw_);
          transform_points.push_back(cv::Point2f(rotated_x + lidar_x_, lidar_y_ - rotated_y));

          const float clockwise_yaw = lidar_yaw_ + deg_to_rad_;
          rotated_x = point.x * std::cos(clockwise_yaw) - point.y * std::sin(clockwise_yaw);
          rotated_y = point.x * std::sin(clockwise_yaw) + point.y * std::cos(clockwise_yaw);
          clockwise_points.push_back(cv::Point2f(rotated_x + lidar_x_, lidar_y_ - rotated_y));

          const float counter_yaw = lidar_yaw_ - deg_to_rad_;
          rotated_x = point.x * std::cos(counter_yaw) - point.y * std::sin(counter_yaw);
          rotated_y = point.x * std::sin(counter_yaw) + point.y * std::cos(counter_yaw);
          counter_points.push_back(cv::Point2f(rotated_x + lidar_x_, lidar_y_ - rotated_y));
        }

        std::vector<cv::Point2f> offsets = {{0, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        std::vector<std::vector<cv::Point2f>> point_sets = {
          transform_points, clockwise_points, counter_points};
        std::vector<float> yaw_offsets = {0.0f, deg_to_rad_, -deg_to_rad_};

        for (size_t i = 0; i < offsets.size(); ++i) {
          for (size_t j = 0; j < point_sets.size(); ++j) {
            int sum = 0;
            for (const auto & point : point_sets[j]) {
              const float px = point.x + offsets[i].x;
              const float py = point.y + offsets[i].y;
              if (px >= 0 && px < map_temp_.cols && py >= 0 && py < map_temp_.rows) {
                sum += map_temp_.at<uchar>(py, px);
              }
            }
            if (sum > max_sum) {
              max_sum = sum;
              best_dx = offsets[i].x;
              best_dy = offsets[i].y;
              best_dyaw = yaw_offsets[j];
            }
          }
        }

        lidar_x_ += best_dx;
        lidar_y_ += best_dy;
        lidar_yaw_ += best_dyaw;

        if (check(lidar_x_, lidar_y_, lidar_yaw_)) {
          break;
        }
      } else {
        break;
      }
    }
  }

  bool check(float x, float y, float yaw)
  {
    if (x == 0.0f && y == 0.0f && yaw == 0.0f) {
      data_queue_.clear();
      return true;
    }

    data_queue_.push_back(std::make_tuple(x, y, yaw));

    const size_t max_size = 10;
    if (data_queue_.size() > max_size) {
      data_queue_.pop_front();
    }

    if (data_queue_.size() == max_size) {
      auto & first = data_queue_.front();
      auto & last = data_queue_.back();

      const float dx = std::abs(std::get<0>(last) - std::get<0>(first));
      const float dy = std::abs(std::get<1>(last) - std::get<1>(first));
      const float dyaw = std::abs(std::get<2>(last) - std::get<2>(first));

      if (dx < 5.0f && dy < 5.0f && dyaw < 5.0f * deg_to_rad_) {
        data_queue_.clear();
        return true;
      }
    }
    return false;
  }

  cv::Mat createGradientMask(int size)
  {
    cv::Mat mask(size, size, CV_8UC1);
    const int center = size / 2;
    for (int y = 0; y < size; y++) {
      for (int x = 0; x < size; x++) {
        const double distance = std::hypot(x - center, y - center);
        const int value = cv::saturate_cast<uchar>(
          255 * std::max(0.0, 1.0 - distance / center));
        mask.at<uchar>(y, x) = value;
      }
    }
    return mask;
  }

  void processMap()
  {
    if (map_cropped_.empty()) {
      return;
    }

    map_temp_ = cv::Mat::zeros(map_cropped_.size(), CV_8UC1);
    cv::Mat gradient_mask = createGradientMask(101);

    for (int y = 0; y < map_cropped_.rows; y++) {
      for (int x = 0; x < map_cropped_.cols; x++) {
        if (map_cropped_.at<uchar>(y, x) == 100) {
          const int left = std::max(0, x - 50);
          const int top = std::max(0, y - 50);
          const int right = std::min(map_cropped_.cols - 1, x + 50);
          const int bottom = std::min(map_cropped_.rows - 1, y + 50);

          cv::Rect roi(left, top, right - left + 1, bottom - top + 1);
          cv::Mat region = map_temp_(roi);

          const int mask_left = 50 - (x - left);
          const int mask_top = 50 - (y - top);
          cv::Rect mask_roi(mask_left, mask_top, roi.width, roi.height);
          cv::Mat mask = gradient_mask(mask_roi);

          cv::max(region, mask, region);
        }
      }
    }
  }

  void poseTf()
  {
    if (scan_count_ == 0) {
      return;
    }
    if (map_cropped_.empty() || map_msg_.data.empty() || map_msg_.info.resolution <= 0.0) {
      return;
    }
    if (!has_odom_transform_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No odom message received on %s; cannot publish %s -> %s",
        odom_topic_.c_str(), global_frame_.c_str(), odom_frame_.c_str());
      return;
    }

    const double full_map_pixel_x = lidar_x_ + map_roi_info_.x_offset;
    const double full_map_pixel_y = lidar_y_ + map_roi_info_.y_offset;

    const double x_in_map_frame =
      full_map_pixel_x * map_msg_.info.resolution + map_msg_.info.origin.position.x;
    const double y_in_map_frame =
      full_map_pixel_y * map_msg_.info.resolution + map_msg_.info.origin.position.y;

    const double yaw_in_map_frame = -lidar_yaw_;

    tf2::Transform map_to_base;
    map_to_base.setOrigin(tf2::Vector3(x_in_map_frame, y_in_map_frame, 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_in_map_frame);
    map_to_base.setRotation(q);

    tf2::Transform odom_to_base_tf2;
    tf2::fromMsg(latest_odom_to_base_.transform, odom_to_base_tf2);
    tf2::Transform map_to_odom = map_to_base * odom_to_base_tf2.inverse();

    geometry_msgs::msg::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp = this->now();
    map_to_odom_msg.header.frame_id = global_frame_;
    map_to_odom_msg.child_frame_id = odom_frame_;
    map_to_odom_msg.transform = tf2::toMsg(map_to_odom);

    tf_broadcaster_->sendTransform(map_to_odom_msg);
  }

  void clearCostmaps()
  {
    for (size_t i = 0; i < clear_costmap_clients_.size(); ++i) {
      auto & client = clear_costmap_clients_[i];
      if (!client->service_is_ready()) {
        RCLCPP_WARN(
          this->get_logger(), "Costmap clear service is not ready: %s",
          clear_service_names_[i].c_str());
        continue;
      }

      auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
      client->async_send_request(request);
      RCLCPP_INFO(this->get_logger(), "Requested costmap clear: %s", clear_service_names_[i].c_str());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarLoc>());
  rclcpp::shutdown();
  return 0;
}
