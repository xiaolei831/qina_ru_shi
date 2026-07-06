#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string.h>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <mutex>
#include <thread>
#include <chrono>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "robot_arm_pick/msg/pick_and_put.hpp"

using namespace std::chrono_literals;
using PickAndPutMsg = robot_arm_pick::msg::PickAndPut;

// ========== 全局共享状态 ==========
static std::mutex g_mtx;
static uint8_t car_state=0;
static int arm_done=1; //置0是表示已经操作完成这个动作，1是表示有新的动作需要完成
static int action_count;
static std::string arm_state="none";
static rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;
static rclcpp::Node::SharedPtr g_node;

// 前向声明
void arm_pick();
void arm_put();
void arm_rotate_put();
void arm_shake_hand();

//接收机械臂运动状态的回调函数
void arm_state_callback(const std_msgs::msg::String::SharedPtr state)
{
  std::lock_guard<std::mutex> lk(g_mtx);
  static std::string last_arm_state;
  arm_state=state->data;
  if(last_arm_state!=arm_state) arm_done=1; //机械臂新状态切换过程检测
  last_arm_state=arm_state;
}


int main(int argc, char **argv)
{ 
    rclcpp::init(argc, argv);
    g_node = std::make_shared<rclcpp::Node>(
        "arm_pick_and_put",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto logger = g_node->get_logger();

    // 启动后台 executor (处理回调)
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(g_node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface arm(g_node, "arm");   
    moveit::planning_interface::MoveGroupInterface hand(g_node, "hand"); 

    arm.setGoalJointTolerance(0.01);

    arm.setMaxVelocityScalingFactor(1);

    hand.setNamedTarget("hand_open"); hand.move(); std::this_thread::sleep_for(1s); //机械爪关闭
    arm.setNamedTarget("arm_uplift"); arm.move(); std::this_thread::sleep_for(1s);  //机械臂回到收起的状态


    auto car_command_pub=g_node->create_publisher<PickAndPutMsg>("car_command",10);//发布机械臂及底盘的运动状态
    joint_states_pub=g_node->create_publisher<sensor_msgs::msg::JointState>("/move_group/fake_controller_joint_states",10);     //发布关节状态
    auto arm_state_sub=g_node->create_subscription<std_msgs::msg::String>("arm_state",10,arm_state_callback);                //订阅机械臂的运动状态
    car_state=0;
    arm_done=1;
   
    RCLCPP_INFO(logger, "arm_pick_and_put 就绪, 等待 arm_state...");
    while(rclcpp::ok())
   {
    std::string current_state;
    int done;
    {
        std::lock_guard<std::mutex> lk(g_mtx);
        current_state = arm_state;
        done = arm_done;
    }
         if (done==1&&current_state=="shake_hand")  { arm_shake_hand(); car_state=4; std::lock_guard<std::mutex> lk(g_mtx); arm_done=0; }//机械臂转动夹爪
    else if (done==1&&current_state=="pick")  { arm_pick(); car_state=1; std::lock_guard<std::mutex> lk(g_mtx); arm_done=0; }           //机械臂抓取色块
    else if (done==1&&current_state=="put")   { arm_put(); car_state=2; std::lock_guard<std::mutex> lk(g_mtx); arm_done=0; }            //机械臂放置色块
    else if (done==1&&current_state=="rotate_put")  { arm_rotate_put(); car_state=3; std::lock_guard<std::mutex> lk(g_mtx); arm_done=0; }//机械臂旋转放置色块
    else if (done==1&&current_state=="no_msg") { car_state=0; std::lock_guard<std::mutex> lk(g_mtx); arm_done=0; }

    PickAndPutMsg msg;
    if(car_state==0)  msg.car_state=0,msg.angle= 0;    //无状态
    if(car_state==1)  msg.car_state=1,msg.angle= 1.57 ;//成功抓取色块后，发出底盘左转的命令
    if(car_state==2)  msg.car_state=2,msg.angle=-1.57 ;//成功放置色块后，发出底盘右转的命令
    if(car_state==3)  msg.car_state=3,msg.angle=0 ;    //机械臂旋转放置色块完毕
    if(car_state==4)  msg.car_state=4,msg.angle= 0;    //机械臂旋转夹爪完毕

    car_command_pub->publish(msg);//将底盘的状态发布出去

    std::this_thread::sleep_for(50ms);
   }
    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}

//一个完整的夹取动作
void arm_pick()
{
    moveit::planning_interface::MoveGroupInterface arm(g_node, "arm");
    moveit::planning_interface::MoveGroupInterface hand(g_node, "hand");
    arm.setGoalJointTolerance(0.01);
    arm.setMaxVelocityScalingFactor(0.1);
    arm.setNamedTarget("arm_clamp");   arm.move();  std::this_thread::sleep_for(1s);
    hand.setNamedTarget("hand_close"); hand.move(); std::this_thread::sleep_for(1s);
    arm.setNamedTarget("arm_uplift");  arm.move();  

}

//一个完整的放置动作
void arm_put()
{
    moveit::planning_interface::MoveGroupInterface arm(g_node, "arm");
    moveit::planning_interface::MoveGroupInterface hand(g_node, "hand");
    arm.setGoalJointTolerance(0.01);
    arm.setMaxAccelerationScalingFactor(1);
    arm.setMaxVelocityScalingFactor(1);
    arm.setNamedTarget("arm_clamp");   arm.move();  std::this_thread::sleep_for(1s);
    hand.setNamedTarget("hand_open");  hand.move(); std::this_thread::sleep_for(1s);
    arm.setNamedTarget("arm_uplift");  arm.move();
}

//一个完整的旋转放置动作
void arm_rotate_put()
{
    moveit::planning_interface::MoveGroupInterface arm(g_node, "arm");
    moveit::planning_interface::MoveGroupInterface hand(g_node, "hand");
    arm.setGoalJointTolerance(0.01);
    arm.setMaxAccelerationScalingFactor(1);
    arm.setMaxVelocityScalingFactor(1);
    arm.setNamedTarget("arm_rotate_uplift");   arm.move();  std::this_thread::sleep_for(1s);
    arm.setNamedTarget("arm_rotate_put");      arm.move();  std::this_thread::sleep_for(1s);
    hand.setNamedTarget("hand_open");          hand.move(); std::this_thread::sleep_for(1s);
    arm.setNamedTarget("arm_rotate_uplift");   arm.move();  std::this_thread::sleep_for(1s);
    arm.setNamedTarget("arm_uplift");          arm.move();  
}

//一个完整的机械臂转动夹爪动作
void arm_shake_hand()
{
    action_count=0;
    float joint_step=0.06;  //机械臂运动的步进值
    float temp=0;
    rclcpp::Rate loop_rate(60); //设置程序执行频率（单位：hz）
    while(rclcpp::ok()){
        if  (action_count>=0 && action_count<16) temp-=joint_step; //执行的第1段动作
        else if(action_count>=16 && action_count<48)  temp+=joint_step; //执行的第2段动作
        else if(action_count>=48 && action_count<80)  temp-=joint_step; //执行的第3段动作
        else if (action_count>=80) temp+=joint_step; //执行的第4段动作
        sensor_msgs::msg::JointState arm_joint_msg;  //定义一个机械臂控制信息的消息数据类型
        //输入当前的ros时间
        arm_joint_msg.header.stamp=g_node->get_clock()->now();
        //输入机械臂臂身的目标关节角度（单位：弧度）
        arm_joint_msg.position.push_back(0);
        arm_joint_msg.position.push_back(0.54);
        arm_joint_msg.position.push_back(1.57);
        arm_joint_msg.position.push_back(1.57);
        arm_joint_msg.position.push_back(temp);
        //输入机械臂夹爪的目标关节角度（单位：弧度）
        arm_joint_msg.position.push_back(0.6);
        arm_joint_msg.position.push_back(-0.6);
        arm_joint_msg.position.push_back(0.6);
        arm_joint_msg.position.push_back(0.6);
        arm_joint_msg.position.push_back(0.6);
        arm_joint_msg.position.push_back(0.6);
        //输入机械臂关节名称
        arm_joint_msg.name.push_back("joint_1");
        arm_joint_msg.name.push_back("joint_2");
        arm_joint_msg.name.push_back("joint_3");
        arm_joint_msg.name.push_back("joint_4");
        arm_joint_msg.name.push_back("joint_5"); 
        arm_joint_msg.name.push_back("joint_6");
        arm_joint_msg.name.push_back("joint_7");
        arm_joint_msg.name.push_back("joint_8");
        arm_joint_msg.name.push_back("joint_9");
        arm_joint_msg.name.push_back("joint_10");
        arm_joint_msg.name.push_back("joint_11");
        //将关节目标角度发布出去
        joint_states_pub->publish(arm_joint_msg);
        action_count+=1;
        if (action_count>=96) break;
        loop_rate.sleep(); //延时等待
    }

}
