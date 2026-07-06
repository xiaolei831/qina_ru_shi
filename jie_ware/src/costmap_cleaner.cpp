#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <rclcpp/rclcpp.hpp>

class CostmapClearer : public rclcpp::Node
{
public:
  CostmapClearer() : Node("costmap_cleaner")
  {
    service_names_ = this->declare_parameter<std::vector<std::string>>(
      "clear_services",
      {
        "/global_costmap/clear_entirely_global_costmap",
        "/local_costmap/clear_entirely_local_costmap",
      });

    for (const auto & service_name : service_names_) {
      clear_costmaps_clients_.push_back(
        this->create_client<nav2_msgs::srv::ClearEntireCostmap>(service_name));
    }

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1),
      std::bind(&CostmapClearer::initialPoseCallback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  std::vector<std::string> service_names_;
  std::vector<rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr> clear_costmaps_clients_;

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr)
  {
    for (size_t i = 0; i < clear_costmaps_clients_.size(); ++i) {
      auto & client = clear_costmaps_clients_[i];
      if (!client->service_is_ready()) {
        RCLCPP_WARN(
          this->get_logger(), "Costmap clear service is not ready: %s",
          service_names_[i].c_str());
        continue;
      }

      auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
      client->async_send_request(request);
      RCLCPP_INFO(this->get_logger(), "Requested costmap clear: %s", service_names_[i].c_str());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapClearer>());
  rclcpp::shutdown();
  return 0;
}
