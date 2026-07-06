#include <cmath>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalize_degrees(double angle)
{
  double normalized = std::fmod(angle, 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }
  return normalized;
}

bool angle_in_range(double angle, double start, double end)
{
  constexpr double epsilon = 1e-4;
  angle = normalize_degrees(angle);
  start = normalize_degrees(start);
  end = normalize_degrees(end);

  if (start <= end) {
    return angle + epsilon >= start && angle <= end + epsilon;
  }

  return angle + epsilon >= start || angle <= end + epsilon;
}
}  // namespace

class ScanMaskFilter : public rclcpp::Node
{
public:
  ScanMaskFilter()
  : Node("scan_mask_filter")
  {
    declare_parameter<std::string>("input_topic", "/scan_raw");
    declare_parameter<std::string>("output_topic", "/scan");
    declare_parameter<double>("mask_angle_start_deg", 135.0);
    declare_parameter<double>("mask_angle_end_deg", 225.0);
    declare_parameter<bool>("enabled", true);
    declare_parameter<bool>("use_inf", true);

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    mask_start_deg_ = get_parameter("mask_angle_start_deg").as_double();
    mask_end_deg_ = get_parameter("mask_angle_end_deg").as_double();
    enabled_ = get_parameter("enabled").as_bool();
    use_inf_ = get_parameter("use_inf").as_bool();

    auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, scan_qos);
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic,
      scan_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        filter_scan(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "scan_mask_filter: %s -> %s, mask %.1fdeg..%.1fdeg, enabled=%s",
      input_topic.c_str(),
      output_topic.c_str(),
      mask_start_deg_,
      mask_end_deg_,
      enabled_ ? "true" : "false");
  }

private:
  void filter_scan(const sensor_msgs::msg::LaserScan & input)
  {
    auto output = input;

    if (enabled_) {
      const float masked_value = use_inf_
        ? std::numeric_limits<float>::infinity()
        : std::numeric_limits<float>::quiet_NaN();

      for (std::size_t i = 0; i < output.ranges.size(); ++i) {
        const double angle_rad = static_cast<double>(output.angle_min) +
          static_cast<double>(i) * static_cast<double>(output.angle_increment);
        const double angle_deg = normalize_degrees(angle_rad * 180.0 / kPi);

        if (angle_in_range(angle_deg, mask_start_deg_, mask_end_deg_)) {
          output.ranges[i] = masked_value;
          if (i < output.intensities.size()) {
            output.intensities[i] = 0.0f;
          }
        }
      }
    }

    publisher_->publish(output);
  }

  double mask_start_deg_ = 135.0;
  double mask_end_deg_ = 225.0;
  bool enabled_ = true;
  bool use_inf_ = true;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanMaskFilter>());
  rclcpp::shutdown();
  return 0;
}
