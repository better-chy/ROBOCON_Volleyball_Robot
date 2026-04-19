#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class DmHitMotorNode : public rclcpp::Node
{
public:
  DmHitMotorNode()
  : Node("dm_hit_motor_node")
  {
    const auto trigger_topic = this->declare_parameter<std::string>("trigger_topic", "/hit_trigger");
    can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");
    shared_can_id_mode_ = this->declare_parameter<bool>("shared_can_id_mode", true);
    shared_motor_id_ = this->declare_parameter<int>("shared_motor_id", 8);
    ready_position_rad_ = this->declare_parameter<double>("ready_position_rad", 0.0);
    strike_position_rad_ = this->declare_parameter<double>("strike_position_rad", 1.0);
    min_position_rad_ = this->declare_parameter<double>("min_position_rad", 0.0);
    max_position_rad_ = this->declare_parameter<double>("max_position_rad", 1.5);

    const auto motor_ids_param =
      this->declare_parameter<std::vector<int64_t>>("motor_ids", {7, 8, 9});
    const auto ready_positions_param =
      this->declare_parameter<std::vector<double>>("ready_positions_rad", {0.0, 0.0, 0.0});
    const auto strike_positions_param =
      this->declare_parameter<std::vector<double>>("strike_positions_rad", {1.0, 1.0, 1.0});

    for (const auto id : motor_ids_param) {
      motor_ids_.push_back(static_cast<int>(id));
    }
    ready_positions_rad_ = ready_positions_param;
    strike_positions_rad_ = strike_positions_param;

    command_id_offset_ = this->declare_parameter<int>("command_id_offset", 0x200);
    position_command_id_offset_ = this->declare_parameter<int>("position_command_id_offset", 0x100);
    strike_velocity_rad_s_ = this->declare_parameter<double>("strike_velocity_rad_s", 10.0);
    return_velocity_rad_s_ = this->declare_parameter<double>("return_velocity_rad_s", 6.0);
    strike_hold_ms_ = this->declare_parameter<int>("strike_hold_ms", 80);
    strike_hold_margin_ms_ = this->declare_parameter<int>("strike_hold_margin_ms", 80);
    cooldown_ms_ = this->declare_parameter<int>("cooldown_ms", 400);
    clear_errors_on_startup_ = this->declare_parameter<bool>("clear_errors_on_startup", true);
    auto_enable_on_trigger_ = this->declare_parameter<bool>("auto_enable_on_trigger", true);
    disable_on_shutdown_ = this->declare_parameter<bool>("disable_on_shutdown", true);

    validateConfiguration();
    can_ready_ = initializeCanSocket();
    last_trigger_time_ = this->now() - rclcpp::Duration::from_seconds(10.0);

    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      trigger_topic, 10,
      std::bind(&DmHitMotorNode::triggerCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "dm_hit_motor_node started");
    if (shared_can_id_mode_) {
      RCLCPP_INFO(
        this->get_logger(),
        "trigger topic: %s shared_can_id=%d strike_hold=%dms margin=%dms cooldown=%dms",
        trigger_topic.c_str(),
        shared_motor_id_,
        strike_hold_ms_,
        strike_hold_margin_ms_,
        cooldown_ms_);
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "trigger topic: %s motors=[%d,%d,%d] strike_hold=%dms margin=%dms cooldown=%dms",
        trigger_topic.c_str(),
        motor_ids_[0],
        motor_ids_[1],
        motor_ids_[2],
        strike_hold_ms_,
        strike_hold_margin_ms_,
        cooldown_ms_);
    }
  }

  ~DmHitMotorNode() override
  {
    if (can_ready_ && disable_on_shutdown_ && motors_enabled_) {
      disableMotors();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    closeCanSocket();
  }

private:
  void validateConfiguration()
  {
    if (shared_can_id_mode_) {
      return;
    }

    if (motor_ids_.size() != 3) {
      RCLCPP_WARN(this->get_logger(), "motor_ids size invalid, fallback to [7,8,9]");
      motor_ids_ = {7, 8, 9};
    }
    if (ready_positions_rad_.size() != 3) {
      RCLCPP_WARN(this->get_logger(), "ready_positions_rad size invalid, fallback to zeros");
      ready_positions_rad_ = {0.0, 0.0, 0.0};
    }
    if (strike_positions_rad_.size() != 3) {
      RCLCPP_WARN(this->get_logger(), "strike_positions_rad size invalid, fallback to ones");
      strike_positions_rad_ = {1.0, 1.0, 1.0};
    }
  }

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

  bool sendCanFrame(const uint32_t can_id, const std::array<uint8_t, 8> & data, uint8_t dlc = 8)
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

  bool sendPositionVelocityFrame(
    const int motor_id,
    const float position_rad,
    const float velocity_rad_s)
  {
    std::array<uint8_t, 8> data{};

    uint32_t position_bits = 0;
    uint32_t velocity_bits = 0;
    std::memcpy(&position_bits, &position_rad, sizeof(position_bits));
    std::memcpy(&velocity_bits, &velocity_rad_s, sizeof(velocity_bits));

    data[0] = static_cast<uint8_t>(position_bits & 0xFF);
    data[1] = static_cast<uint8_t>((position_bits >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>((position_bits >> 16) & 0xFF);
    data[3] = static_cast<uint8_t>((position_bits >> 24) & 0xFF);
    data[4] = static_cast<uint8_t>(velocity_bits & 0xFF);
    data[5] = static_cast<uint8_t>((velocity_bits >> 8) & 0xFF);
    data[6] = static_cast<uint8_t>((velocity_bits >> 16) & 0xFF);
    data[7] = static_cast<uint8_t>((velocity_bits >> 24) & 0xFF);

    return sendCanFrame(static_cast<uint32_t>(motor_id + position_command_id_offset_), data, 8);
  }

  void clearErrors()
  {
    if (shared_can_id_mode_) {
      sendMotorCommand(shared_motor_id_, 0xFB);
      return;
    }

    for (const auto motor_id : motor_ids_) {
      sendMotorCommand(motor_id, 0xFB);
    }
  }

  void enableMotors()
  {
    if (shared_can_id_mode_) {
      sendMotorCommand(shared_motor_id_, 0xFC);
      return;
    }

    for (const auto motor_id : motor_ids_) {
      sendMotorCommand(motor_id, 0xFC);
    }
  }

  void disableMotors()
  {
    if (shared_can_id_mode_) {
      sendMotorCommand(shared_motor_id_, 0xFD);
      return;
    }

    for (const auto motor_id : motor_ids_) {
      sendMotorCommand(motor_id, 0xFD);
    }
  }

  void sendPositions(const std::vector<double> & positions, const double velocity_rad_s)
  {
    if (shared_can_id_mode_) {
      sendPositionVelocityFrame(
        shared_motor_id_,
        static_cast<float>(clampPosition(positions.empty() ? 0.0 : positions.front())),
        static_cast<float>(velocity_rad_s));
      return;
    }

    for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
      sendPositionVelocityFrame(
        motor_ids_[i],
        static_cast<float>(clampPosition(positions[i])),
        static_cast<float>(velocity_rad_s));
    }
  }

  double clampPosition(const double position_rad) const
  {
    return std::clamp(position_rad, min_position_rad_, max_position_rad_);
  }

  double activeReadyPositionRad() const
  {
    if (!ready_positions_rad_.empty()) {
      return clampPosition(ready_positions_rad_.front());
    }
    return clampPosition(ready_position_rad_);
  }

  double activeStrikePositionRad() const
  {
    if (!strike_positions_rad_.empty()) {
      return clampPosition(strike_positions_rad_.front());
    }
    return clampPosition(strike_position_rad_);
  }

  int effectiveStrikeHoldMs() const
  {
    const double distance = std::abs(activeStrikePositionRad() - activeReadyPositionRad());
    const double velocity = std::max(std::abs(strike_velocity_rad_s_), 1e-3);
    const int travel_ms = static_cast<int>((distance / velocity) * 1000.0);
    return std::max(strike_hold_ms_, travel_ms + strike_hold_margin_ms_);
  }

  void triggerCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }

    const auto elapsed_ms = (this->now() - last_trigger_time_).nanoseconds() / 1000000LL;
    if (sequence_running_ || elapsed_ms < cooldown_ms_) {
      return;
    }

    sequence_running_ = true;
    last_trigger_time_ = this->now();

    if (can_ready_ && auto_enable_on_trigger_ && !motors_enabled_) {
      if (clear_errors_on_startup_) {
        clearErrors();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      enableMotors();
      motors_enabled_ = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (can_ready_) {
      const auto hold_ms = effectiveStrikeHoldMs();
      sendPositions(strike_positions_rad_, strike_velocity_rad_s_);
      RCLCPP_INFO(
        this->get_logger(),
        "hit target strike=%.3f ready=%.3f v_strike=%.3f v_return=%.3f hold=%dms",
        activeStrikePositionRad(),
        activeReadyPositionRad(),
        strike_velocity_rad_s_,
        return_velocity_rad_s_,
        hold_ms);
      std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
      sendPositions(ready_positions_rad_, return_velocity_rad_s_);
    }

    sequence_running_ = false;
    RCLCPP_INFO(this->get_logger(), "hit sequence executed");
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;

  std::string can_interface_{"can0"};
  bool shared_can_id_mode_{true};
  int shared_motor_id_{8};
  double ready_position_rad_{0.0};
  double strike_position_rad_{1.0};
  double min_position_rad_{0.0};
  double max_position_rad_{1.5};
  std::vector<int> motor_ids_{};
  std::vector<double> ready_positions_rad_{};
  std::vector<double> strike_positions_rad_{};
  int command_id_offset_{0x200};
  int position_command_id_offset_{0x100};
  double strike_velocity_rad_s_{10.0};
  double return_velocity_rad_s_{6.0};
  int strike_hold_ms_{80};
  int strike_hold_margin_ms_{80};
  int cooldown_ms_{400};
  bool clear_errors_on_startup_{true};
  bool auto_enable_on_trigger_{true};
  bool disable_on_shutdown_{true};

  int can_socket_{-1};
  bool can_ready_{false};
  bool motors_enabled_{false};
  bool sequence_running_{false};
  rclcpp::Time last_trigger_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DmHitMotorNode>());
  rclcpp::shutdown();
  return 0;
}
