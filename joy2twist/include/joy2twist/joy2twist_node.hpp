#ifndef JOY2TWIST_JOY2TWIST_NODE_HPP_
#define JOY2TWIST_JOY2TWIST_NODE_HPP_

#include <map>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "joy2twist/joy_input.hpp"

namespace joy2twist
{
using MsgJoy = sensor_msgs::msg::Joy;
using MsgTwist = geometry_msgs::msg::Twist;
using MsgTwistStamped = geometry_msgs::msg::TwistStamped;
using MsgAckermannDrive = ackermann_msgs::msg::AckermannDrive;
using MsgAckermannDriveStamped = ackermann_msgs::msg::AckermannDriveStamped;
using MsgBool = std_msgs::msg::Bool;
using SrvTrigger = std_srvs::srv::Trigger;

struct RawInputIndex
{
  std::string angular_z;
  std::string linear_x;
  std::string linear_y;
  std::string linear_z_up;
  std::string linear_z_down;

  std::string dead_man_switch;
  std::string fast_mode;
  std::string slow_mode;

  std::string e_stop_reset;
  std::string e_stop_trigger;
  std::string enable_e_stop_reset;
};

struct InputIndex
{
  JoyInput angular_z;
  JoyInput linear_x;
  JoyInput linear_y;
  JoyInput linear_z_up;
  JoyInput linear_z_down;

  JoyInput dead_man_switch;
  JoyInput fast_mode;
  JoyInput slow_mode;

  JoyInput e_stop_reset;
  JoyInput e_stop_trigger;
  JoyInput enable_e_stop_reset;
};

class Joy2TwistNode : public rclcpp::Node
{
public:
  Joy2TwistNode();

private:
  void declare_parameters();
  void load_parameters();
  void parse_joy_inputs(const RawInputIndex & raw_input_index);

  // Returns raw axis or button value
  float get_joy_input(const MsgJoy::SharedPtr joy_msg, const JoyInput & joy_input) const;

  // Returns either binary button state or quantized axis value
  bool get_joy_input_as_btn(const MsgJoy::SharedPtr joy_msg, const JoyInput & joy_input) const;

  void e_stop_cb(const std::shared_ptr<MsgBool> bool_msg);
  void joy_cb(const std::shared_ptr<MsgJoy> joy_msg);
  void convert_joy_to_twist(const std::shared_ptr<MsgJoy> joy_msg, MsgTwist & twist_msg);
  void convert_joy_to_ackermann(const std::shared_ptr<MsgJoy> joy_msg, MsgAckermannDrive & ackermann_msg);
  std::pair<float, float> determine_velocity_factor(const std::shared_ptr<MsgJoy> joy_msg);
  void publish_twist(const MsgTwist & twist_msg);
  void publish_ackermann(const MsgAckermannDrive & ackermann_msg);
  void call_trigger_service(const rclcpp::Client<SrvTrigger>::SharedPtr & client) const;
  void trigger_service_cb(
    const rclcpp::Client<SrvTrigger>::SharedFuture & future,
    const std::string & service_name) const;
  void handle_e_stop(const std::shared_ptr<MsgJoy> joy_msg);

  std::map<std::string, float> linear_velocity_factors_;
  std::map<std::string, float> angular_velocity_factors_;

  InputIndex input_index_;
  bool driving_mode_;
  bool e_stop_present_;
  bool e_stop_state_;
  bool cmd_vel_stamped_;
  bool use_ackermann_;
  bool ackermann_stamped_;
  float wheelbase_;
  std::string e_stop_topic_;
  std::string e_stop_reset_srv_;
  std::string e_stop_trigger_srv_;

  rclcpp::Subscription<MsgBool>::SharedPtr e_stop_sub_;
  rclcpp::Subscription<MsgJoy>::SharedPtr joy_sub_;
  rclcpp::Publisher<MsgTwist>::SharedPtr twist_pub_;
  rclcpp::Publisher<MsgTwistStamped>::SharedPtr twist_stamped_pub_;
  rclcpp::Publisher<MsgAckermannDrive>::SharedPtr ackermann_pub_;
  rclcpp::Publisher<MsgAckermannDriveStamped>::SharedPtr ackermann_stamped_pub_;
  rclcpp::Client<SrvTrigger>::SharedPtr e_stop_reset_client_;
  rclcpp::Client<SrvTrigger>::SharedPtr e_stop_trigger_client_;
};

static constexpr char kFast[]{"fast"};
static constexpr char kRegular[]{"regular"};
static constexpr char kSlow[]{"slow"};

static constexpr float kAxisToButtonDeadzone = 0.75f;
}  // namespace joy2twist

#endif  // JOY2TWIST_JOY2TWIST_NODE_HPP_
