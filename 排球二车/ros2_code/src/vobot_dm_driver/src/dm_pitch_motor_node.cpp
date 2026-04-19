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
#include "std_msgs/msg/float32.hpp"

class DmPitchMotorNode : public rclcpp::Node
{
public:
  DmPitchMotorNode()
  : Node("dm_pitch_motor_node")
  {
    const auto pitch_target_topic =
      this->declare_parameter<std::string>("pitch_target_topic", "/pitch_target_rad");
    can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");

    motor_id_ = this->declare_parameter<int>("motor_id", 5);
    command_id_offset_ = this->declare_parameter<int>("command_id_offset", 0x200);
    position_command_id_offset_ = this->declare_parameter<int>("position_command_id_offset", 0x100);

    min_pitch_rad_ = this->declare_parameter<double>("min_pitch_rad", -0.35);
    max_pitch_rad_ = this->declare_parameter<double>("max_pitch_rad", 0.52);
    tracking_velocity_rad_s_ =
      this->declare_parameter<double>("tracking_velocity_rad_s", 2.0);

    clear_errors_on_startup_ = this->declare_parameter<bool>("clear_errors_on_startup", true);
    auto_enable_on_first_target_ =
      this->declare_parameter<bool>("auto_enable_on_first_target", true);
    disable_on_shutdown_ = this->declare_parameter<bool>("disable_on_shutdown", false);

    can_ready_ = initializeCanSocket();

    pitch_target_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      pitch_target_topic, 10,
      std::bind(&DmPitchMotorNode::pitchTargetCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "dm_pitch_motor_node started");
    RCLCPP_INFO(this->get_logger(), "subscribing topic: %s", pitch_target_topic.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "pitch motor: id=%d can=%s cmd_offset=0x%03X pos_offset=0x%03X",
      motor_id_,
      can_interface_.c_str(),
      static_cast<unsigned int>(command_id_offset_),
      static_cast<unsigned int>(position_command_id_offset_));
    RCLCPP_INFO(
      this->get_logger(),
      "pitch range(rad): [%.2f, %.2f] tracking_velocity=%.2f",
      min_pitch_rad_,
      max_pitch_rad_,
      tracking_velocity_rad_s_);
  }

  ~DmPitchMotorNode() override
  {
    if (can_ready_ && disable_on_shutdown_) {
      disableMotor();
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

  bool sendMotorCommand(const uint8_t command_byte)
  {
    std::array<uint8_t, 8> data{};
    data.fill(0xFF);
    data[7] = command_byte;
    return sendCanFrame(static_cast<uint32_t>(motor_id_ + command_id_offset_), data);
  }

  bool clearErrors()
  {
    return sendMotorCommand(0xFB);
  }

  bool enableMotor()
  {
    return sendMotorCommand(0xFC);
  }

  bool disableMotor()
  {
    return sendMotorCommand(0xFD);
  }

  bool sendPositionVelocityFrame(const float position_rad, const float velocity_rad_s)
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

    return sendCanFrame(
      static_cast<uint32_t>(motor_id_ + position_command_id_offset_),
      data,
      8);
  }

  void pitchTargetCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    const float clamped_target = static_cast<float>(
      std::clamp(static_cast<double>(msg->data), min_pitch_rad_, max_pitch_rad_));

    if (can_ready_ && auto_enable_on_first_target_ && !motor_enabled_) {
      if (clear_errors_on_startup_) {
        clearErrors();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      motor_enabled_ = enableMotor();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (can_ready_) {
      sendPositionVelocityFrame(clamped_target, static_cast<float>(tracking_velocity_rad_s_));
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "pitch target(rad): %.3f motor_id=%d enabled=%s",
      clamped_target,
      motor_id_,
      motor_enabled_ ? "true" : "false");
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pitch_target_sub_;

  std::string can_interface_{"can0"};
  int motor_id_{5};
  int command_id_offset_{0x200};
  int position_command_id_offset_{0x100};
  double min_pitch_rad_{-0.35};
  double max_pitch_rad_{0.52};
  double tracking_velocity_rad_s_{2.0};
  bool clear_errors_on_startup_{true};
  bool auto_enable_on_first_target_{true};
  bool disable_on_shutdown_{false};

  int can_socket_{-1};
  bool can_ready_{false};
  bool motor_enabled_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DmPitchMotorNode>());
  rclcpp::shutdown();
  return 0;
}
