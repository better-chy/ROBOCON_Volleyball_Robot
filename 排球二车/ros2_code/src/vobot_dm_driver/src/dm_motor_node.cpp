#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "dm_msgs/msg/wheel_speeds.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class DmMotorNode : public rclcpp::Node
{
public:
  struct MotorTarget
  {
    int motor_id;
    double target_speed_rad_s;
    bool enable;
  };

  DmMotorNode()
  : Node("dm_motor_node")
  {
    const auto wheel_speeds_topic =
      this->declare_parameter<std::string>("wheel_speeds_topic", "/wheel_speeds");
    can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");

    auto motor_ids =
      this->declare_parameter<std::vector<int64_t>>("motor_ids", {2, 3, 1, 4});
    auto motor_directions =
      this->declare_parameter<std::vector<double>>("motor_directions", {1.0, 1.0, 1.0, 1.0});

    speed_command_scale_ = this->declare_parameter<double>("speed_command_scale", 1.0);
    launch_boost_gain_ = this->declare_parameter<double>("launch_boost_gain", 1.0);
    launch_boost_threshold_ = this->declare_parameter<double>("launch_boost_threshold", 0.0);
    max_motor_speed_ = this->declare_parameter<double>("max_motor_speed", 20.0);
    accel_limit_rad_s2_ = this->declare_parameter<double>("accel_limit_rad_s2", 40.0);
    decel_limit_rad_s2_ = this->declare_parameter<double>("decel_limit_rad_s2", 12.0);

    command_id_offset_ = this->declare_parameter<int>("command_id_offset", 0x200);
    clear_errors_on_startup_ = this->declare_parameter<bool>("clear_errors_on_startup", true);

    auto_enable_on_startup_ = this->declare_parameter<bool>("auto_enable_on_startup", true);
    enable_on_first_command_ = this->declare_parameter<bool>("enable_on_first_command", true);
    disable_on_shutdown_ = this->declare_parameter<bool>("disable_on_shutdown", true);
    command_timeout_ms_ = this->declare_parameter<int>("command_timeout_ms", 300);
    watchdog_period_ms_ = this->declare_parameter<int>("watchdog_period_ms", 50);

    if (motor_ids.size() != 4) {
      RCLCPP_WARN(
        this->get_logger(),
        "motor_ids size is %zu, fallback to [2, 3, 1, 4]",
        motor_ids.size());
      motor_ids = {2, 3, 1, 4};
    }

    if (motor_directions.size() != 4) {
      RCLCPP_WARN(
        this->get_logger(),
        "motor_directions size is %zu, fallback to [1, 1, 1, 1]",
        motor_directions.size());
      motor_directions = {1.0, 1.0, 1.0, 1.0};
    }

    for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
      motor_ids_[i] = static_cast<int>(motor_ids[i]);
      motor_directions_[i] = motor_directions[i];
    }

    can_ready_ = initializeCanSocket();
    if (can_ready_) {
      if (clear_errors_on_startup_) {
        clearErrors();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }

      if (auto_enable_on_startup_) {
        motors_enabled_ = enableMotors();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }

    last_command_time_ = this->now();
    timeout_stop_sent_ = true;

    wheel_speeds_sub_ = this->create_subscription<dm_msgs::msg::WheelSpeeds>(
      wheel_speeds_topic, 10,
      std::bind(&DmMotorNode::wheelSpeedsCallback, this, std::placeholders::_1));

    watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(watchdog_period_ms_),
      std::bind(&DmMotorNode::watchdogCallback, this));

    RCLCPP_INFO(this->get_logger(), "dm_motor_node started");
    RCLCPP_INFO(this->get_logger(), "subscribing topic: %s", wheel_speeds_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "CAN interface: %s", can_interface_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "wheel->motor map: FL->%d FR->%d RL->%d RR->%d",
      motor_ids_[0], motor_ids_[1], motor_ids_[2], motor_ids_[3]);
    RCLCPP_INFO(
      this->get_logger(),
      "driver shaping: scale=%.2f launch_boost_gain=%.2f threshold=%.2f max_speed=%.2f cmd_offset=0x%03X",
      speed_command_scale_,
      launch_boost_gain_,
      launch_boost_threshold_,
      max_motor_speed_,
      static_cast<unsigned int>(command_id_offset_));
    RCLCPP_INFO(
      this->get_logger(),
      "rate limits: accel=%.2f rad/s^2 decel=%.2f rad/s^2",
      accel_limit_rad_s2_,
      decel_limit_rad_s2_);
    RCLCPP_INFO(
      this->get_logger(),
      "safety: clear_errors=%s auto_enable=%s first_cmd_enable=%s timeout=%dms shutdown_disable=%s",
      clear_errors_on_startup_ ? "true" : "false",
      auto_enable_on_startup_ ? "true" : "false",
      enable_on_first_command_ ? "true" : "false",
      command_timeout_ms_,
      disable_on_shutdown_ ? "true" : "false");
  }

  ~DmMotorNode() override
  {
    if (can_ready_) {
      sendZeroVelocityToAll();
      if (disable_on_shutdown_) {
        disableMotors();
      }
    }

    closeCanSocket();
  }

private:
  bool initializeCanSocket()
  {
    can_socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(
        this->get_logger(), "failed to create CAN socket on %s: %s",
        can_interface_.c_str(), std::strerror(errno));
      return false;
    }

    struct ifreq ifr {};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_interface_.c_str());
    if (::ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(
        this->get_logger(), "failed to query interface %s: %s",
        can_interface_.c_str(), std::strerror(errno));
      closeCanSocket();
      return false;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(
        this->get_logger(), "failed to bind CAN interface %s: %s",
        can_interface_.c_str(), std::strerror(errno));
      closeCanSocket();
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "SocketCAN connected on %s", can_interface_.c_str());
    return true;
  }

  void closeCanSocket()
  {
    if (can_socket_ >= 0) {
      ::close(can_socket_);
      can_socket_ = -1;
    }
  }

  bool sendCanFrame(
    const uint32_t can_id,
    const std::array<uint8_t, 8> & data,
    const uint8_t dlc = 8)
  {
    if (!can_ready_ || can_socket_ < 0) {
      return false;
    }

    struct can_frame frame {};
    frame.can_id = can_id;
    frame.can_dlc = dlc;
    std::copy(data.begin(), data.end(), frame.data);

    const auto bytes_written = ::write(can_socket_, &frame, sizeof(frame));
    if (bytes_written != static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "failed to send CAN frame id=0x%03X: %s",
        can_id, std::strerror(errno));
      return false;
    }

    return true;
  }

  bool sendMotorCommand(const int motor_id, const uint8_t command_byte)
  {
    std::array<uint8_t, 8> data{};
    data.fill(0xFF);
    data[7] = command_byte;
    return sendCanFrame(static_cast<uint32_t>(motor_id + command_id_offset_), data);
  }

  bool clearErrors()
  {
    bool ok = true;
    for (const auto motor_id : motor_ids_) {
      ok = sendMotorCommand(motor_id, 0xFB) && ok;
    }

    if (ok) {
      RCLCPP_INFO(this->get_logger(), "all motors clear-error command sent");
    }
    return ok;
  }

  bool enableMotors()
  {
    bool ok = true;
    for (const auto motor_id : motor_ids_) {
      ok = sendMotorCommand(motor_id, 0xFC) && ok;
    }

    if (ok) {
      RCLCPP_INFO(this->get_logger(), "all motors enabled");
    }
    return ok;
  }

  bool disableMotors()
  {
    bool ok = true;
    for (const auto motor_id : motor_ids_) {
      ok = sendMotorCommand(motor_id, 0xFD) && ok;
    }

    if (ok) {
      RCLCPP_INFO(this->get_logger(), "all motors disabled");
    }
    return ok;
  }

  bool sendVelocityFrame(const MotorTarget & target)
  {
    std::array<uint8_t, 8> data{};

    const float velocity = static_cast<float>(target.target_speed_rad_s);
    uint32_t velocity_bits = 0;
    std::memcpy(&velocity_bits, &velocity, sizeof(velocity_bits));

    data[0] = static_cast<uint8_t>(velocity_bits & 0xFF);
    data[1] = static_cast<uint8_t>((velocity_bits >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>((velocity_bits >> 16) & 0xFF);
    data[3] = static_cast<uint8_t>((velocity_bits >> 24) & 0xFF);
    return sendCanFrame(static_cast<uint32_t>(target.motor_id + command_id_offset_), data, 4);
  }

  bool sendTargets(const std::array<MotorTarget, 4> & motor_targets)
  {
    bool ok = true;
    for (const auto & target : motor_targets) {
      ok = sendVelocityFrame(target) && ok;
    }
    return ok;
  }

  double limitRate(
    const double current_speed,
    const double target_speed,
    const double dt) const
  {
    if (dt <= 0.0) {
      return current_speed;
    }

    const double diff = target_speed - current_speed;
    if (std::abs(diff) < 1e-6) {
      return target_speed;
    }

    bool is_decelerating = std::abs(target_speed) < std::abs(current_speed);
    if ((current_speed > 0.0 && target_speed < 0.0) ||
      (current_speed < 0.0 && target_speed > 0.0))
    {
      is_decelerating = true;
    }

    const double limit = is_decelerating ? decel_limit_rad_s2_ : accel_limit_rad_s2_;
    if (limit <= 0.0) {
      return target_speed;
    }

    const double max_step = limit * dt;
    if (std::abs(diff) <= max_step) {
      return target_speed;
    }

    return current_speed + std::copysign(max_step, diff);
  }

  std::array<MotorTarget, 4> buildMotorTargets(const dm_msgs::msg::WheelSpeeds & msg) const
  {
    const std::array<double, 4> wheel_targets = {
      msg.front_left,
      msg.front_right,
      msg.rear_left,
      msg.rear_right
    };

    std::array<MotorTarget, 4> motor_targets{};
    for (std::size_t i = 0; i < motor_targets.size(); ++i) {
      double target_speed = wheel_targets[i] * motor_directions_[i] * speed_command_scale_;

      // Give small non-zero commands a configurable "kick" for faster launch.
      if (std::abs(target_speed) > 0.0 && std::abs(target_speed) < launch_boost_threshold_) {
        target_speed *= launch_boost_gain_;
      }

      if (max_motor_speed_ > 0.0) {
        if (target_speed > max_motor_speed_) {
          target_speed = max_motor_speed_;
        } else if (target_speed < -max_motor_speed_) {
          target_speed = -max_motor_speed_;
        }
      }

      motor_targets[i] = MotorTarget{
        motor_ids_[i],
        msg.enable ? target_speed : 0.0,
        msg.enable
      };
    }

    return motor_targets;
  }

  std::array<MotorTarget, 4> rampMotorTargets(
    const std::array<MotorTarget, 4> & desired_targets)
  {
    auto now = this->now();

    double dt = 0.0;
    if (last_ramp_time_.nanoseconds() > 0) {
      dt = (now - last_ramp_time_).seconds();
    }

    if (dt <= 0.0 || dt > 0.5) {
      dt = static_cast<double>(watchdog_period_ms_) / 1000.0;
    }

    std::array<MotorTarget, 4> applied_targets = desired_targets;
    for (std::size_t i = 0; i < desired_targets.size(); ++i) {
      applied_motor_speeds_[i] = limitRate(
        applied_motor_speeds_[i],
        desired_targets[i].target_speed_rad_s,
        dt);
      applied_targets[i].target_speed_rad_s = applied_motor_speeds_[i];
    }

    last_ramp_time_ = now;
    return applied_targets;
  }

  void sendZeroVelocityToAll()
  {
    std::array<MotorTarget, 4> zero_targets{};
    for (std::size_t i = 0; i < zero_targets.size(); ++i) {
      zero_targets[i] = MotorTarget{motor_ids_[i], 0.0, false};
    }

    sendTargets(zero_targets);
  }

  void wheelSpeedsCallback(const dm_msgs::msg::WheelSpeeds::SharedPtr msg)
  {
    const auto desired_targets = buildMotorTargets(*msg);

    if (can_ready_ && enable_on_first_command_ && !motors_enabled_ && msg->enable) {
      if (clear_errors_on_startup_) {
        clearErrors();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      motors_enabled_ = enableMotors();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    const auto motor_targets = rampMotorTargets(desired_targets);

    if (can_ready_) {
      sendTargets(motor_targets);
    }

    last_command_time_ = this->now();
    timeout_stop_sent_ = false;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "motor targets(rad/s): id%d=%.3f id%d=%.3f id%d=%.3f id%d=%.3f enable=%s",
      motor_targets[0].motor_id, motor_targets[0].target_speed_rad_s,
      motor_targets[1].motor_id, motor_targets[1].target_speed_rad_s,
      motor_targets[2].motor_id, motor_targets[2].target_speed_rad_s,
      motor_targets[3].motor_id, motor_targets[3].target_speed_rad_s,
      motor_targets[0].enable ? "true" : "false");
  }

  void watchdogCallback()
  {
    if (!can_ready_ || command_timeout_ms_ <= 0) {
      return;
    }

    const auto elapsed_ms = (this->now() - last_command_time_).nanoseconds() / 1000000LL;
    if (elapsed_ms < command_timeout_ms_) {
      return;
    }

    std::array<MotorTarget, 4> zero_targets{};
    for (std::size_t i = 0; i < zero_targets.size(); ++i) {
      zero_targets[i] = MotorTarget{motor_ids_[i], 0.0, false};
    }

    const auto ramped_zero_targets = rampMotorTargets(zero_targets);
    sendTargets(ramped_zero_targets);

    bool all_zero = true;
    for (const auto speed : applied_motor_speeds_) {
      if (std::abs(speed) > 0.05) {
        all_zero = false;
        break;
      }
    }

    if (!timeout_stop_sent_) {
      RCLCPP_WARN(
        this->get_logger(),
        "command timeout after %lld ms, ramping motor speeds toward zero",
        elapsed_ms);
    }

    timeout_stop_sent_ = all_zero;
  }

  rclcpp::Subscription<dm_msgs::msg::WheelSpeeds>::SharedPtr wheel_speeds_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::array<int, 4> motor_ids_{};
  std::array<double, 4> motor_directions_{};

  std::string can_interface_{"can0"};
  double speed_command_scale_{1.0};
  double launch_boost_gain_{1.0};
  double launch_boost_threshold_{0.0};
  double max_motor_speed_{20.0};
  double accel_limit_rad_s2_{40.0};
  double decel_limit_rad_s2_{12.0};
  int command_id_offset_{0x200};
  bool clear_errors_on_startup_{true};
  bool auto_enable_on_startup_{true};
  bool enable_on_first_command_{true};
  bool disable_on_shutdown_{true};
  int command_timeout_ms_{300};
  int watchdog_period_ms_{50};

  int can_socket_{-1};
  bool can_ready_{false};
  bool motors_enabled_{false};
  bool timeout_stop_sent_{true};
  std::array<double, 4> applied_motor_speeds_{0.0, 0.0, 0.0, 0.0};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ramp_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DmMotorNode>());
  rclcpp::shutdown();
  return 0;
}
