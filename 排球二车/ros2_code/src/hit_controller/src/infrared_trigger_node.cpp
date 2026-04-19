#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gpiod.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class InfraredTriggerNode : public rclcpp::Node
{
public:
  InfraredTriggerNode()
  : Node("infrared_trigger_node")
  {
    const auto trigger_topic = this->declare_parameter<std::string>("trigger_topic", "/hit_trigger");
    gpio_chip_path_ = this->declare_parameter<std::string>("gpio_chip", "/dev/gpiochip0");
    active_low_ = this->declare_parameter<bool>("active_low", false);
    poll_period_ms_ = this->declare_parameter<int>("poll_period_ms", 10);
    retrigger_lockout_ms_ = this->declare_parameter<int>("retrigger_lockout_ms", 400);

    const auto gpio_lines_param =
      this->declare_parameter<std::vector<int64_t>>("gpio_lines", {0, 1, 2, 3});
    for (const auto line : gpio_lines_param) {
      gpio_lines_.push_back(static_cast<unsigned int>(line));
    }

    trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>(trigger_topic, 10);

    gpio_ready_ = initializeGpio();
    last_trigger_time_ = this->now() - rclcpp::Duration::from_seconds(10.0);
    line_states_initialized_ = false;

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(poll_period_ms_),
      std::bind(&InfraredTriggerNode::pollInputs, this));

    RCLCPP_INFO(this->get_logger(), "infrared_trigger_node started");
    RCLCPP_INFO(
      this->get_logger(),
      "trigger topic: %s gpio_chip=%s active_low=%s poll=%dms lockout=%dms",
      trigger_topic.c_str(),
      gpio_chip_path_.c_str(),
      active_low_ ? "true" : "false",
      poll_period_ms_,
      retrigger_lockout_ms_);
  }

  ~InfraredTriggerNode() override
  {
    cleanupGpio();
  }

private:
  bool initializeGpio()
  {
    chip_ = gpiod_chip_open(gpio_chip_path_.c_str());
    if (chip_ == nullptr) {
      RCLCPP_ERROR(
        this->get_logger(),
        "failed to open gpio chip %s",
        gpio_chip_path_.c_str());
      return false;
    }

    for (const auto line_index : gpio_lines_) {
      gpiod_line * line = gpiod_chip_get_line(chip_, line_index);
      if (line == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "failed to get gpio line %u", line_index);
        cleanupGpio();
        return false;
      }

      if (gpiod_line_request_input_flags(line, "infrared_trigger_node", 0) < 0) {
        RCLCPP_ERROR(this->get_logger(), "failed to request gpio line %u as input", line_index);
        cleanupGpio();
        return false;
      }

      lines_.push_back(line);
    }

    RCLCPP_INFO(this->get_logger(), "GPIO ready with %zu infrared inputs", lines_.size());
    return true;
  }

  void cleanupGpio()
  {
    for (auto * line : lines_) {
      if (line != nullptr) {
        gpiod_line_release(line);
      }
    }
    lines_.clear();

    if (chip_ != nullptr) {
      gpiod_chip_close(chip_);
      chip_ = nullptr;
    }
  }

  void publishTrigger()
  {
    std_msgs::msg::Bool msg;
    msg.data = true;
    trigger_pub_->publish(msg);
    last_trigger_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "infrared trigger fired");
  }

  void pollInputs()
  {
    if (!gpio_ready_) {
      return;
    }

    std::vector<int> current_states{};
    current_states.reserve(lines_.size());

    for (std::size_t i = 0; i < lines_.size(); ++i) {
      const int value = gpiod_line_get_value(lines_[i]);
      if (value < 0) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "failed to read gpio line %u",
          gpio_lines_[i]);
        return;
      }
      current_states.push_back(isActiveValue(value) ? 1 : 0);
    }

    // Prime the previous-state cache so startup-active inputs don't immediately fire.
    if (!line_states_initialized_) {
      last_line_states_ = current_states;
      line_states_initialized_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "infrared initial states: [%d, %d, %d, %d]",
        current_states.size() > 0 ? current_states[0] : -1,
        current_states.size() > 1 ? current_states[1] : -1,
        current_states.size() > 2 ? current_states[2] : -1,
        current_states.size() > 3 ? current_states[3] : -1);
      return;
    }

    const auto elapsed_ms = (this->now() - last_trigger_time_).nanoseconds() / 1000000LL;
    if (elapsed_ms < retrigger_lockout_ms_) {
      last_line_states_ = current_states;
      return;
    }

    for (std::size_t i = 0; i < current_states.size(); ++i) {
      if (last_line_states_[i] == 0 && current_states[i] == 1) {
        publishTrigger();
        break;
      }
    }

    last_line_states_ = current_states;
  }

  bool isActiveValue(const int raw_value) const
  {
    return active_low_ ? (raw_value == 0) : (raw_value == 1);
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string gpio_chip_path_{"/dev/gpiochip0"};
  std::vector<unsigned int> gpio_lines_{};
  bool active_low_{false};
  int poll_period_ms_{10};
  int retrigger_lockout_ms_{400};

  bool gpio_ready_{false};
  gpiod_chip * chip_{nullptr};
  std::vector<gpiod_line *> lines_{};
  std::vector<int> last_line_states_{};
  bool line_states_initialized_{false};
  rclcpp::Time last_trigger_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InfraredTriggerNode>());
  rclcpp::shutdown();
  return 0;
}
