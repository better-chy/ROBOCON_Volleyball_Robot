#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "dm_msgs/msg/wheel_speeds.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double deg2rad(const double degrees)
{
  return degrees * kPi / 180.0;
}
}  // namespace

class BaseControllerNode : public rclcpp::Node
{
public:
  BaseControllerNode()
  : Node("base_controller_node")
  {
    const auto cmd_vel_topic =
      this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto wheel_speeds_topic =
      this->declare_parameter<std::string>("wheel_speeds_topic", "/wheel_speeds");

    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.0635);
    half_length_ = this->declare_parameter<double>("half_length", 0.2667);
    half_width_ = this->declare_parameter<double>("half_width", 0.2667);
    max_wheel_speed_ = this->declare_parameter<double>("max_wheel_speed", 20.0);
    publish_enable_ = this->declare_parameter<bool>("publish_enable", true);

    auto wheel_angles_deg =
      this->declare_parameter<std::vector<double>>(
      "wheel_angles_deg", {-45.0, -135.0, 45.0, 135.0});

    if (wheel_radius_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "wheel_radius <= 0, fallback to 0.0635 m");
      wheel_radius_ = 0.0635;
    }

    if (wheel_angles_deg.size() != 4) {
      RCLCPP_WARN(
        this->get_logger(),
        "wheel_angles_deg size is %zu, fallback to default 4-wheel layout",
        wheel_angles_deg.size());
      wheel_angles_deg = {-45.0, -135.0, 45.0, 135.0};
    }

    for (std::size_t i = 0; i < wheel_angles_.size(); ++i) {
      wheel_angles_[i] = deg2rad(wheel_angles_deg[i]);
    }

    wheel_speeds_pub_ =
      this->create_publisher<dm_msgs::msg::WheelSpeeds>(wheel_speeds_topic, 10);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      std::bind(&BaseControllerNode::cmdVelCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "base_controller_node started");
    RCLCPP_INFO(
      this->get_logger(),
      "topics: %s -> %s",
      cmd_vel_topic.c_str(), wheel_speeds_topic.c_str());
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const auto wheel_speeds = computeWheelSpeeds(
      msg->linear.x,
      msg->linear.y,
      msg->angular.z);

    dm_msgs::msg::WheelSpeeds wheel_msg;
    wheel_msg.front_left = static_cast<float>(wheel_speeds[0]);
    wheel_msg.front_right = static_cast<float>(wheel_speeds[1]);
    wheel_msg.rear_left = static_cast<float>(wheel_speeds[2]);
    wheel_msg.rear_right = static_cast<float>(wheel_speeds[3]);
    wheel_msg.enable = publish_enable_;

    wheel_speeds_pub_->publish(wheel_msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "wheel speeds(rad/s): FL=%.3f FR=%.3f RL=%.3f RR=%.3f",
      wheel_speeds[0], wheel_speeds[1], wheel_speeds[2], wheel_speeds[3]);
  }

  std::array<double, 4> computeWheelSpeeds(
    const double vx,
    const double vy,
    const double wz) const
  {
    const std::array<std::array<double, 2>, 4> wheel_positions = {{
      {half_length_, half_width_},
      {half_length_, -half_width_},
      {-half_length_, half_width_},
      {-half_length_, -half_width_}
    }};

    std::array<double, 4> wheel_speeds{};

    for (std::size_t i = 0; i < wheel_speeds.size(); ++i) {
      const double theta = wheel_angles_[i];
      const double x_i = wheel_positions[i][0];
      const double y_i = wheel_positions[i][1];

      const double wheel_linear_speed =
        std::cos(theta) * vx +
        std::sin(theta) * vy +
        (-y_i * std::cos(theta) + x_i * std::sin(theta)) * wz;

      double wheel_angular_speed = wheel_linear_speed / wheel_radius_;

      if (max_wheel_speed_ > 0.0) {
        wheel_angular_speed = std::clamp(
          wheel_angular_speed, -max_wheel_speed_, max_wheel_speed_);
      }

      wheel_speeds[i] = wheel_angular_speed;
    }

    return wheel_speeds;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<dm_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_pub_;

  double wheel_radius_{0.0635};
  double half_length_{0.2667};
  double half_width_{0.2667};
  double max_wheel_speed_{20.0};
  bool publish_enable_{true};
  std::array<double, 4> wheel_angles_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseControllerNode>());
  rclcpp::shutdown();
  return 0;
}
