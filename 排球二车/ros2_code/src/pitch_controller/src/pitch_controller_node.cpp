#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class PitchControllerNode : public rclcpp::Node
{
public:
  PitchControllerNode()
  : Node("pitch_controller_node")
  {
    const auto joy_topic = this->declare_parameter<std::string>("joy_topic", "/joy");
    const auto pitch_target_topic =
      this->declare_parameter<std::string>("pitch_target_topic", "/pitch_target_rad");

    up_axis_ = this->declare_parameter<int>("up_axis", 2);
    down_axis_ = this->declare_parameter<int>("down_axis", 5);
    command_deadband_ = this->declare_parameter<double>("command_deadband", 0.05);

    min_pitch_rad_ = this->declare_parameter<double>("min_pitch_rad", -0.35);
    max_pitch_rad_ = this->declare_parameter<double>("max_pitch_rad", 0.52);
    target_pitch_rad_ = this->declare_parameter<double>("initial_target_rad", 0.0);
    pitch_rate_rad_s_ = this->declare_parameter<double>("pitch_rate_rad_s", 0.8);

    publish_period_ms_ = this->declare_parameter<int>("publish_period_ms", 20);

    target_pitch_rad_ = std::clamp(target_pitch_rad_, min_pitch_rad_, max_pitch_rad_);

    pitch_target_pub_ = this->create_publisher<std_msgs::msg::Float32>(pitch_target_topic, 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic, 10,
      std::bind(&PitchControllerNode::joyCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_period_ms_),
      std::bind(&PitchControllerNode::timerCallback, this));

    last_update_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "pitch_controller_node started");
    RCLCPP_INFO(
      this->get_logger(),
      "topics: %s -> %s",
      joy_topic.c_str(),
      pitch_target_topic.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "trigger axes: up=%d down=%d rate=%.2f rad/s range=[%.2f, %.2f]",
      up_axis_,
      down_axis_,
      pitch_rate_rad_s_,
      min_pitch_rad_,
      max_pitch_rad_);
  }

private:
  static double axisToPress(const double axis_value)
  {
    const double clamped = std::clamp(axis_value, -1.0, 1.0);
    return (1.0 - clamped) * 0.5;
  }

  double getAxisValue(const int axis_index) const
  {
    if (!has_joy_ || axis_index < 0 || static_cast<std::size_t>(axis_index) >= last_axes_.size()) {
      return 1.0;
    }
    return last_axes_[static_cast<std::size_t>(axis_index)];
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_axes_ = msg->axes;
    has_joy_ = true;
  }

  void timerCallback()
  {
    const auto now = this->now();
    double dt = (now - last_update_time_).seconds();
    if (dt <= 0.0 || dt > 0.5) {
      dt = static_cast<double>(publish_period_ms_) / 1000.0;
    }
    last_update_time_ = now;

    const double up_press = axisToPress(getAxisValue(up_axis_));
    const double down_press = axisToPress(getAxisValue(down_axis_));
    double pitch_cmd = up_press - down_press;

    if (std::abs(pitch_cmd) < command_deadband_) {
      pitch_cmd = 0.0;
    }

    if (std::abs(pitch_cmd) < 1e-6) {
      return;
    }

    target_pitch_rad_ += pitch_cmd * pitch_rate_rad_s_ * dt;
    target_pitch_rad_ = std::clamp(target_pitch_rad_, min_pitch_rad_, max_pitch_rad_);

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(target_pitch_rad_);
    pitch_target_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "pitch target(rad): %.3f up=%.2f down=%.2f cmd=%.2f",
      target_pitch_rad_,
      up_press,
      down_press,
      pitch_cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int up_axis_{2};
  int down_axis_{5};
  int publish_period_ms_{20};
  double command_deadband_{0.05};
  double min_pitch_rad_{-0.35};
  double max_pitch_rad_{0.52};
  double target_pitch_rad_{0.0};
  double pitch_rate_rad_s_{0.8};

  bool has_joy_{false};
  std::vector<float> last_axes_{};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PitchControllerNode>());
  rclcpp::shutdown();
  return 0;
}
