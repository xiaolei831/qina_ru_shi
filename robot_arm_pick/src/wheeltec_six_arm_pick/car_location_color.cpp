#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "robot_arm_pick/msg/pick_and_put.hpp"
#include "robot_arm_pick/msg/six_arm_position.hpp"

using namespace std::chrono_literals;
using PickAndPutMsg = robot_arm_pick::msg::PickAndPut;
using SixArmPosMsg  = robot_arm_pick::msg::SixArmPosition;

class CarLocationColor : public rclcpp::Node
{
public:
  CarLocationColor() : Node("car_location_color")
  {
    //参数声明
    this->declare_parameter("x_p", 0.15); //x轴方向的pid参数
    this->declare_parameter("x_d", 0.04);
    this->declare_parameter("y_p", 0.15); //y轴方向的pid参数
    this->declare_parameter("y_d", 0.04);
    this->declare_parameter("z_p", 0.3);  //z轴方向的pid参数
    this->declare_parameter("z_d", 0.03);
    this->declare_parameter("color_location_x", 0.0909); //色块的x轴方位的目标位置
    this->declare_parameter("color_location_y", 0.2924); //色块的y轴方位的目标位置
    this->declare_parameter("car_mode", std::string("mini_4wd_six_arm"));//获取小车的类型
    this->declare_parameter("rotate_mode", std::string("holder"));//获取夹取色块后旋转方式

    x_p = this->get_parameter("x_p").as_double();
    x_d = this->get_parameter("x_d").as_double();
    y_p = this->get_parameter("y_p").as_double();
    y_d = this->get_parameter("y_d").as_double();
    z_p = this->get_parameter("z_p").as_double();
    z_d = this->get_parameter("z_d").as_double();
    color_location_x = this->get_parameter("color_location_x").as_double();
    color_location_y = this->get_parameter("color_location_y").as_double();
    car_mode = this->get_parameter("car_mode").as_string();
    rotate_mode = this->get_parameter("rotate_mode").as_string();

    arm_state_pub = this->create_publisher<std_msgs::msg::String>("arm_state", 10);
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    car_command_sub = this->create_subscription<PickAndPutMsg>(
        "car_command", 10, std::bind(&CarLocationColor::car_command_callback, this, std::placeholders::_1));
    color_location_sub = this->create_subscription<SixArmPosMsg>(
        "object_tracker/current_position", 10, std::bind(&CarLocationColor::color_location_callback, this, std::placeholders::_1));
    if (rotate_mode == "chassis") {
      car_pose_sub = this->create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10, std::bind(&CarLocationColor::car_pose_callback, this, std::placeholders::_1));
    }

    target_angle = 0; car_state = -1; location_flag = 0; move_flag = 0; shake_hand = 0; //标志位初始化
    shake_hand_done = {0, 0, 0};

    // 主控制循环定时器
    timer_ = this->create_wall_timer(20ms, std::bind(&CarLocationColor::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "car_location_color_node_init_successful");
  }

  ~CarLocationColor()
  {
    RCLCPP_INFO(this->get_logger(), "car_location_color_node_close");
  }

private:
  //接收小车状态的回调函数
  void car_command_callback(const PickAndPutMsg::SharedPtr msg)
  {
    car_state = msg->car_state;
    target_angle = msg->angle;
  }

  //获取色块位置的回调函数，在函数中做pid计算，将色块定位到可夹取位置，当未非目标色块时执行一次机械臂转动夹爪动作
  void color_location_callback(const SixArmPosMsg::SharedPtr msg)
  {
    static int count = 0;

    //检测到目标色块
    if (msg->correct && car_state == 0 && shake_hand == 0) {
      //识别到目标色块，将error标志位置零
      error_flag = 0;

      /*******************小车底盘定位色块*******************/
      //mini_mec
      if (car_mode == "mini_mec_six_arm")
      {
        if (location_flag == 0 && car_state == 0) {
          distance_y = msg->angle_x;
          distance_x = msg->angle_y;
          target_liner_x = PID_control_x();
          target_liner_y = PID_control_y();
        }
        //当pid控制下的小车底盘速度小于0.005时开始计数，计数达到150则认为底盘已定位好位置
        if ((abs_float(target_liner_x) < 0.003) && (abs_float(target_liner_y) < 0.005) && car_state == 0) count++;
        else count = 0;
        if (count > 150) { location_flag = 1; count = 0; }//色块定位完成
        if (location_flag == 0) target_angular_z = 0; //色块定位过程中只需要x和y轴的线速度
        if (location_flag == 1) { target_liner_x = 0; target_liner_y = 0; } //色块定位完成后底盘不再定位，防止抖动
      }

      //mini_tank,mini_4wd
      else if ((car_mode == "mini_tank_six_arm") || (car_mode == "mini_4wd_six_arm")) {
        if (location_flag == 0 && car_state == 0) {
          distance_x = msg->angle_y;
          angular_z = msg->angle_x;
          target_liner_x = PID_control_x();
          target_angular_z = PID_control_z();
        }
        if ((abs_float(target_liner_x) < 0.003) && (abs_float(target_angular_z) < 0.005) && car_state == 0) count++;
        else count = 0;
        if (count >= 150) { location_flag = 1; count = 0; }//色块定位完成
        if (location_flag == 0) target_liner_y = 0; //色块定位只需要x轴线速度及z轴角速度
        if (location_flag == 1) {
          target_liner_x = 0;
          target_angular_z = 0; //色块定位完成后底盘不再定位，防止抖动
        }
        else if (location_flag == 1 && car_state != 0) {
          target_liner_x = 0;
        }
      }

      //将底盘速度发布出去
      cmd_vel_publish();
    }

    //检测到非目标色块,仅当小车处于识别色块状态且识别到非目标色块执行
    else if (!msg->correct && car_state == 0 && location_flag == 0 && shake_hand == 0) {
      if (msg->color == "yellow" && shake_hand_done[0] == 0) { shake_hand = 1; shake_hand_done[0] = 1; }
      if (msg->color == "blue" && shake_hand_done[1] == 0)   { shake_hand = 1; shake_hand_done[1] = 1; }
      if (msg->color == "green" && shake_hand_done[2] == 0)  { shake_hand = 1; shake_hand_done[2] = 1; }
    }
  }

  //获取底盘位置的回调函数，利用odom话题来做底盘夹取到色块后旋转多少角度的计算
  void car_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    static float last_target_angle = 0, target_position_z = 0;

    car_position_x = msg->pose.pose.position.x;//获取底盘的位置信息（编码器积分获得）
    car_position_y = msg->pose.pose.position.y;
    car_position_z = msg->pose.pose.position.z;

    if (last_target_angle != target_angle) target_position_z = car_position_z + target_angle;//状态发生变化，获取需要转多少角度
    if (car_state == 1)//夹取到色块后
    {     //给上0.1的允许误差范围
          if (target_position_z <= (car_position_z - 0.1)) { target_angular_z = -0.6; move_flag = 1; } //以0.6rad/s的速度旋转
     else if (target_position_z >= (car_position_z + 0.1)) { target_angular_z =  0.6; move_flag = 1; }
     else { target_angular_z = 0; move_flag = 2; } //夹取色块后自转完成
     last_target_angle = target_angle;
     cmd_vel_publish();//将底盘速度发布出去
    }

    if (car_state == 2)//放置完色块后
    {     //给上0.1的允许误差范围
          if (target_position_z <= (car_position_z - 0.1)) { target_angular_z = -0.6; move_flag = 3; } //以0.6rad/s的速度旋转
     else if (target_position_z >= (car_position_z + 0.1)) { target_angular_z =  0.6; move_flag = 3; }
     else { target_angular_z = 0; move_flag = 4; } //放置色块后自转完成
     last_target_angle = target_angle;
     cmd_vel_publish();//将底盘速度发布出去
    }
    last_target_angle = target_angle;
  }

  //运动底盘速度话题发布函数
  void cmd_vel_publish()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = target_liner_x;
    msg.linear.y = target_liner_y;
    msg.angular.z = target_angular_z;
    cmd_vel_pub->publish(msg);
  }

  //机械臂状态的话题发布函数
  void arm_state_publish()
  {
    std_msgs::msg::String msg;
    msg.data = "last_target_angle";

    //当机械臂转动夹爪动作标志位为1时发送"shake_hand"命令，执行转动夹爪动作
    if (shake_hand)                                                    msg.data = "shake_hand";
    //机械臂云台旋转并放置物块，之后回到原位
    else if ((car_state == 1) && (rotate_mode == "holder"))            msg.data = "rotate_put";
    //如果底盘对色块定位完成，那么发布机械臂夹取的命令
    else if ((location_flag == 1) && (move_flag < 2))                  msg.data = "pick";
    //底盘自转前，如果是夹取了东西的，那么就执行机械臂放置命令
    else if ((move_flag == 2 || move_flag == 3) && (rotate_mode == "chassis")) msg.data = "put";
    //其他情况则发送"no_msg"
    else                                                               msg.data = "no_msg";

    //将机械臂的命令发布出去
    arm_state_pub->publish(msg);
  }

  //底盘在x轴方向（前后）定位色块的PID计算
  float PID_control_x()
  {
    static float last_error = 0, output = 0;
    pid_x.error = distance_x - (color_location_x);
    output = x_d * last_error + x_p * pid_x.error;
    last_error = pid_x.error;
    return output;
  }

  //底盘在y轴方向（左右）定位色块的PID计算
  float PID_control_y()
  {
    static float last_error = 0, output = 0;
    pid_y.error = distance_y - (color_location_y);
    output = y_d * last_error + y_p * pid_y.error;
    last_error = pid_y.error;
    return output;
  }

  //底盘在z轴方向（旋转）定位色块的PID计算
  float PID_control_z()
  {
    static float last_error = 0, output = 0;
    pid_z.error = angular_z - (color_location_y);
    output = z_d * last_error + z_p * pid_z.error;
    last_error = pid_z.error;
    return output;
  }

  //取绝对值函数
  float abs_float(float input)
  {
    if (input < 0) return -input;
    else return input;
  }

  //定时器循环执行的内容函数
  void control_loop()
  {
    //当小车执行完一套夹取放置色块动作后将标志位清零
    if (move_flag == 4 || car_state == 3) {
      car_state = 0; location_flag = 0; move_flag = 0;
      for (size_t i = 0; i < shake_hand_done.size(); i++) {
        shake_hand_done[i] = 0;
      }
    }

    //当小车执行完一次转动夹爪动作后将标志位清零
    if (car_state == 4) {
      car_state = 0; location_flag = 0; move_flag = 0; shake_hand = 0;
    }

    //发布机械臂状态的命令
    arm_state_publish();
    //error标志位为0，则将error计置零
    if (error_flag == 0) error_count = 0;
    //当error标志位为1且处于定位色块阶段，则开始error计数
    else if (error_flag == 1 && location_flag == 0) error_count++;
    error_flag = 1;
    //当error计数达到10000时，将底盘速度置零防止小车乱跑
    if (error_count > 10000) {
      target_liner_x = 0;
      target_liner_y = 0;
      target_angular_z = 0;
      cmd_vel_publish();
      error_count = 0;
    }
  }

  // ---- PID 结构体 ----
  struct PID_NUMBER {
    float error = 0;
    float integral = 0;
    float diff = 0;
  };

  // ---- 成员变量 ----
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_state_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Subscription<PickAndPutMsg>::SharedPtr car_command_sub;
  rclcpp::Subscription<SixArmPosMsg>::SharedPtr color_location_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr car_pose_sub;
  rclcpp::TimerBase::SharedPtr timer_;

  float distance_y = 0, distance_x = 0, angular_z = 0;
  uint8_t move_flag = 0, location_flag = 0;
  float car_position_x = 0, car_position_y = 0, car_position_z = 0;
  double x_p = 0, x_d = 0, y_p = 0, y_d = 0, z_p = 0, z_d = 0;
  double color_location_x = 0, color_location_y = 0;
  PID_NUMBER pid_x, pid_y, pid_z;
  float target_liner_x = 0, target_liner_y = 0, target_angular_z = 0;
  float target_angle = 0;
  uint8_t car_state = 0, shake_hand = 0;
  std::string car_mode, rotate_mode;
  std::vector<int> shake_hand_done;
  long int error_flag = 1, error_count = 0;
};

//主函数
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); //节点初始化
  auto node = std::make_shared<CarLocationColor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
