#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

class JoyHitTriggerNode : public rclcpp::Node
{
public:
  JoyHitTriggerNode()
  : Node("joy_hit_trigger_node")
  {
    const auto joy_topic = this->declare_parameter<std::string>("joy_topic", "/joy");
    const auto trigger_topic = this->declare_parameter<std::string>("trigger_topic", "/hit_trigger");
    hit_button_index_ = this->declare_parameter<int>("hit_button_index", 0);
    retrigger_lockout_ms_ = this->declare_parameter<int>("retrigger_lockout_ms", 400);

    trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>(trigger_topic, 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic, 10,
      std::bind(&JoyHitTriggerNode::joyCallback, this, std::placeholders::_1));

    last_trigger_time_ = this->now() - rclcpp::Duration::from_seconds(10.0);

    RCLCPP_INFO(this->get_logger(), "joy_hit_trigger_node started");
    RCLCPP_INFO(
      this->get_logger(),
      "topics: %s -> %s button=%d lockout=%dms",
      joy_topic.c_str(),
      trigger_topic.c_str(),
      hit_button_index_,
      retrigger_lockout_ms_);
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (hit_button_index_ < 0 || hit_button_index_ >= static_cast<int>(msg->buttons.size())) {
      return;
    }

    const bool pressed = msg->buttons[hit_button_index_] == 1;
    const auto elapsed_ms = (this->now() - last_trigger_time_).nanoseconds() / 1000000LL;

    if (!last_button_pressed_ && pressed && elapsed_ms >= retrigger_lockout_ms_) {
      std_msgs::msg::Bool trigger_msg;
      trigger_msg.data = true;
      trigger_pub_->publish(trigger_msg);
      last_trigger_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "joy hit trigger fired");
    }

    last_button_pressed_ = pressed;
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_;

  int hit_button_index_{0};
  int retrigger_lockout_ms_{400};
  bool last_button_pressed_{false};
  rclcpp::Time last_trigger_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyHitTriggerNode>());
  rclcpp::shutdown();
  return 0;
}
