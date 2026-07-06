#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class CLidarFilter : public rclcpp::Node
{
public:
  CLidarFilter() : Node("lidar_filter_node")
  {
    source_topic_name_ = this->declare_parameter<std::string>("source_topic", "/scan");
    pub_topic_name_ = this->declare_parameter<std::string>("pub_topic", "/scan_filtered");
    outlier_threshold_ = this->declare_parameter<double>("outlier_threshold", 0.1);

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      pub_topic_name_, rclcpp::SensorDataQoS());
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      source_topic_name_, rclcpp::SensorDataQoS(),
      std::bind(&CLidarFilter::lidarCallback, this, std::placeholders::_1));
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::string source_topic_name_;
  std::string pub_topic_name_;
  double outlier_threshold_;

  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    const int n_ranges = static_cast<int>(scan->ranges.size());

    if (n_ranges < 3) {
      scan_pub_->publish(*scan);
      return;
    }

    sensor_msgs::msg::LaserScan new_scan = *scan;

    for (int i = 1; i < n_ranges - 1; ++i) {
      const float prev_range = new_scan.ranges[i - 1];
      const float current_range = new_scan.ranges[i];
      const float next_range = new_scan.ranges[i + 1];

      const bool current_valid =
        std::isfinite(current_range) &&
        current_range >= new_scan.range_min &&
        current_range <= new_scan.range_max;

      if (!current_valid) {
        continue;
      }

      if (
        std::abs(current_range - prev_range) > outlier_threshold_ &&
        std::abs(current_range - next_range) > outlier_threshold_)
      {
        new_scan.ranges[i] = std::numeric_limits<float>::infinity();
        if (!new_scan.intensities.empty() && i < static_cast<int>(new_scan.intensities.size())) {
          new_scan.intensities[i] = 0.0f;
        }
      }
    }

    scan_pub_->publish(new_scan);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CLidarFilter>());
  rclcpp::shutdown();
  return 0;
}
