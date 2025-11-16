#include "joy2twist/joy2twist_node.hpp"

namespace joy2twist
{
using std::placeholders::_1;

Joy2TwistNode::Joy2TwistNode() : Node("joy2twist_node")
{
  using namespace std::placeholders;

  declare_parameters();
  load_parameters();

  joy_sub_ = create_subscription<MsgJoy>(
    "joy", rclcpp::SensorDataQoS(), std::bind(&Joy2TwistNode::joy_cb, this, _1));

  if (use_ackermann_) {
    if (ackermann_stamped_) {
      ackermann_stamped_pub_ = create_publisher<MsgAckermannDriveStamped>(
        "ackermann_cmd", rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable());
    } else {
      ackermann_pub_ = create_publisher<MsgAckermannDrive>(
        "ackermann_cmd", rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable());
    }
  } else {
    if (cmd_vel_stamped_) {
      twist_stamped_pub_ = create_publisher<MsgTwistStamped>(
        "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable());
    } else {
      twist_pub_ = create_publisher<MsgTwist>(
        "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable());
    }
  }

  if (e_stop_present_) {
    e_stop_sub_ = this->create_subscription<MsgBool>(
      e_stop_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&Joy2TwistNode::e_stop_cb, this, _1));
    e_stop_reset_client_ = this->create_client<SrvTrigger>(e_stop_reset_srv_);
    e_stop_trigger_client_ = this->create_client<SrvTrigger>(e_stop_trigger_srv_);
  }

  RCLCPP_INFO(get_logger(), "Initialized node!");
}

void Joy2TwistNode::declare_parameters()
{
  this->declare_parameter<bool>("cmd_vel_stamped", false);
  this->declare_parameter<bool>("use_ackermann", false);
  this->declare_parameter<bool>("ackermann_stamped", false);
  this->declare_parameter<float>("wheelbase", 0.335);

  this->declare_parameter<float>("linear_velocity_factor.fast", 1.0);
  this->declare_parameter<float>("linear_velocity_factor.regular", 0.5);
  this->declare_parameter<float>("linear_velocity_factor.slow", 0.2);
  this->declare_parameter<float>("angular_velocity_factor.fast", 1.0);
  this->declare_parameter<float>("angular_velocity_factor.regular", 0.5);
  this->declare_parameter<float>("angular_velocity_factor.slow", 0.2);

  this->declare_parameter<bool>("e_stop.present", false);
  this->declare_parameter<std::string>("e_stop.topic", "e_stop");
  this->declare_parameter<std::string>("e_stop.reset_srv", "e_stop_reset");
  this->declare_parameter<std::string>("e_stop.trigger_srv", "e_stop_trigger");

  this->declare_parameter<std::string>("input_index_map.axis.angular_z", "A2");
  this->declare_parameter<std::string>("input_index_map.axis.linear_x", "A1");
  this->declare_parameter<std::string>("input_index_map.axis.linear_y", "A0");
  this->declare_parameter<std::string>("input_index_map.axis.linear_z_up", "A5");
  this->declare_parameter<std::string>("input_index_map.axis.linear_z_down", "A6");
  this->declare_parameter<std::string>("input_index_map.dead_man_switch", "B4");
  this->declare_parameter<std::string>("input_index_map.fast_mode", "B7");
  this->declare_parameter<std::string>("input_index_map.slow_mode", "B5");
  this->declare_parameter<std::string>("input_index_map.e_stop_reset", "B1");
  this->declare_parameter<std::string>("input_index_map.e_stop_trigger", "B2");
  this->declare_parameter<std::string>("input_index_map.enable_e_stop_reset", "B6");
}

void Joy2TwistNode::load_parameters()
{
  this->get_parameter<bool>("cmd_vel_stamped", cmd_vel_stamped_);
  this->get_parameter<bool>("use_ackermann", use_ackermann_);
  this->get_parameter<bool>("ackermann_stamped", ackermann_stamped_);
  this->get_parameter<float>("wheelbase", wheelbase_);

  this->get_parameter<float>("linear_velocity_factor.fast", linear_velocity_factors_[kFast]);
  this->get_parameter<float>("linear_velocity_factor.regular", linear_velocity_factors_[kRegular]);
  this->get_parameter<float>("linear_velocity_factor.slow", linear_velocity_factors_[kSlow]);
  this->get_parameter<float>("angular_velocity_factor.fast", angular_velocity_factors_[kFast]);
  this->get_parameter<float>(
    "angular_velocity_factor.regular", angular_velocity_factors_[kRegular]);
  this->get_parameter<float>("angular_velocity_factor.slow", angular_velocity_factors_[kSlow]);

  this->get_parameter<bool>("e_stop.present", e_stop_present_);
  this->get_parameter<std::string>("e_stop.topic", e_stop_topic_);
  this->get_parameter<std::string>("e_stop.reset_srv", e_stop_reset_srv_);
  this->get_parameter<std::string>("e_stop.trigger_srv", e_stop_trigger_srv_);

  RawInputIndex raw_input_index{};

  this->get_parameter<std::string>("input_index_map.axis.angular_z", raw_input_index.angular_z);
  this->get_parameter<std::string>("input_index_map.axis.linear_x", raw_input_index.linear_x);
  this->get_parameter<std::string>("input_index_map.axis.linear_y", raw_input_index.linear_y);
  this->get_parameter<std::string>("input_index_map.axis.linear_z_up", raw_input_index.linear_z_up);
  this->get_parameter<std::string>("input_index_map.axis.linear_z_down", raw_input_index.linear_z_down);
  this->get_parameter<std::string>(
    "input_index_map.dead_man_switch", raw_input_index.dead_man_switch);
  this->get_parameter<std::string>("input_index_map.fast_mode", raw_input_index.fast_mode);
  this->get_parameter<std::string>("input_index_map.slow_mode", raw_input_index.slow_mode);
  this->get_parameter<std::string>("input_index_map.e_stop_reset", raw_input_index.e_stop_reset);
  this->get_parameter<std::string>(
    "input_index_map.e_stop_trigger", raw_input_index.e_stop_trigger);
  this->get_parameter<std::string>(
    "input_index_map.enable_e_stop_reset", raw_input_index.enable_e_stop_reset);

  parse_joy_inputs(raw_input_index);
}

void Joy2TwistNode::parse_joy_inputs(const RawInputIndex & raw_input_index)
{
  input_index_.angular_z = JoyInput::from_string(raw_input_index.angular_z);
  input_index_.linear_x = JoyInput::from_string(raw_input_index.linear_x);
  input_index_.linear_y = JoyInput::from_string(raw_input_index.linear_y);
  input_index_.linear_z_up = JoyInput::from_string(raw_input_index.linear_z_up);
  input_index_.linear_z_down = JoyInput::from_string(raw_input_index.linear_z_down);
  input_index_.dead_man_switch = JoyInput::from_string(raw_input_index.dead_man_switch);
  input_index_.fast_mode = JoyInput::from_string(raw_input_index.fast_mode);
  input_index_.slow_mode = JoyInput::from_string(raw_input_index.slow_mode);
  input_index_.e_stop_reset = JoyInput::from_string(raw_input_index.e_stop_reset);
  input_index_.e_stop_trigger = JoyInput::from_string(raw_input_index.e_stop_trigger);
  input_index_.enable_e_stop_reset = JoyInput::from_string(raw_input_index.enable_e_stop_reset);
}

float Joy2TwistNode::get_joy_input(
  const MsgJoy::SharedPtr joy_msg, const JoyInput & joy_input) const
{
  float negation_factor = joy_input.is_inverted ? -1.0f : 1.0f;

  if (joy_input.type == JoyInput::Type::AXIS) {
    return joy_msg->axes.at(joy_input.index) * negation_factor;
  }

  if (joy_input.type == JoyInput::Type::BUTTON) {
    return static_cast<float>(joy_msg->buttons.at(joy_input.index)) * negation_factor;
  }

  throw std::invalid_argument("Invalid JoyInput type");
}

bool Joy2TwistNode::get_joy_input_as_btn(
  const MsgJoy::SharedPtr joy_msg, const JoyInput & joy_input) const
{
  if (joy_input.type == JoyInput::Type::AXIS) {
    auto axis = joy_msg->axes.at(joy_input.index);

    if (joy_input.is_inverted) {
      return axis < -kAxisToButtonDeadzone ? 1 : 0;
    }
    return axis > kAxisToButtonDeadzone ? 1 : 0;
  }

  if (joy_input.type == JoyInput::Type::BUTTON) {
    auto value = static_cast<bool>(joy_msg->buttons.at(joy_input.index));
    return joy_input.is_inverted ? !value : value;
  }

  throw std::invalid_argument("Invalid JoyInput type");
}

void Joy2TwistNode::e_stop_cb(const MsgBool::SharedPtr bool_msg) { e_stop_state_ = bool_msg->data; }

void Joy2TwistNode::joy_cb(const MsgJoy::SharedPtr joy_msg)
{
  handle_e_stop(joy_msg);

  if (get_joy_input_as_btn(joy_msg, input_index_.dead_man_switch)) {
    driving_mode_ = true;
    if (use_ackermann_) {
      MsgAckermannDrive ackermann_msg;
      convert_joy_to_ackermann(joy_msg, ackermann_msg);
      publish_ackermann(ackermann_msg);
    } else {
      MsgTwist twist_msg;
      convert_joy_to_twist(joy_msg, twist_msg);
      publish_twist(twist_msg);
    }
  } else if (driving_mode_) {
    driving_mode_ = false;
    if (use_ackermann_) {
      publish_ackermann(MsgAckermannDrive());
    } else {
      publish_twist(MsgTwist());
    }
  }
}

void Joy2TwistNode::convert_joy_to_twist(const MsgJoy::SharedPtr joy_msg, MsgTwist & twist_msg)
{
  float linear_velocity_factor{}, angular_velocity_factor{};
  std::tie(linear_velocity_factor, angular_velocity_factor) = determine_velocity_factor(joy_msg);

  twist_msg.angular.z = angular_velocity_factor * get_joy_input(joy_msg, input_index_.angular_z);
  twist_msg.linear.x = linear_velocity_factor * get_joy_input(joy_msg, input_index_.linear_x);
  twist_msg.linear.y = linear_velocity_factor * get_joy_input(joy_msg, input_index_.linear_y);
  
  // Handle linear.z for drone up/down control
  float z_up_raw = get_joy_input(joy_msg, input_index_.linear_z_up);
  float z_down_raw = get_joy_input(joy_msg, input_index_.linear_z_down);
  
  // Debug output - always show to help diagnose the issue
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Raw trigger values - RT (up): %.3f, LT (down): %.3f", z_up_raw, z_down_raw);
  
  // Xbox triggers range from -1.0 (not pressed) to +1.0 (fully pressed)
  // Simple approach: normalize to 0.0-1.0, then apply desired output range
  
  // Normalize triggers: -1.0 -> 0.0, +1.0 -> 1.0
  float z_up_normalized = -(z_up_raw - 1.0f) * 0.5f; // Convert -1.0 to +1.0 -> 0.0 to 1.0
  float z_down_normalized = (z_down_raw - 1.0f) * 0.5f; // Convert -1.0 to +1.0 -> 0.0 to 1.0
  
  // Apply a deadzone to avoid drift when triggers are not pressed
  const float trigger_deadzone = 0.1f;
  if (z_up_normalized < trigger_deadzone) z_up_normalized = 0.0f;
  if (z_down_normalized > trigger_deadzone) z_down_normalized = 0.0f;
  
  // Calculate final Z velocity:
  // RT pressed: +0.5, LT pressed: -0.5, nothing pressed: 0.0
  float z_velocity = z_up_normalized + z_down_normalized;
  
  // Debug processed values
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Normalized - RT: %.3f, LT: %.3f, Z_vel: %.3f", z_up_normalized, z_down_normalized, z_velocity);
  
  // Apply velocity factor and set final Z velocity
  twist_msg.linear.z = linear_velocity_factor * z_velocity;
}

std::pair<float, float> Joy2TwistNode::determine_velocity_factor(const MsgJoy::SharedPtr joy_msg)
{
  float linear_velocity_factor = linear_velocity_factors_.at(kRegular);
  float angular_velocity_factor = angular_velocity_factors_.at(kRegular);
  if (
    get_joy_input_as_btn(joy_msg, input_index_.slow_mode) &&
    !get_joy_input_as_btn(joy_msg, input_index_.fast_mode)) {
    linear_velocity_factor = linear_velocity_factors_.at(kSlow);
    angular_velocity_factor = angular_velocity_factors_.at(kSlow);
  } else if (
    get_joy_input_as_btn(joy_msg, input_index_.fast_mode) &&
    !get_joy_input_as_btn(joy_msg, input_index_.slow_mode)) {
    linear_velocity_factor = linear_velocity_factors_.at(kFast);
    angular_velocity_factor = angular_velocity_factors_.at(kFast);
  }
  return std::make_pair(linear_velocity_factor, angular_velocity_factor);
}

void Joy2TwistNode::publish_twist(const MsgTwist & twist_msg)
{
  if (cmd_vel_stamped_) {
    MsgTwistStamped twist_stamped_msg;
    twist_stamped_msg.header.stamp = this->get_clock()->now();
    twist_stamped_msg.twist = twist_msg;
    twist_stamped_pub_->publish(twist_stamped_msg);
  } else {
    twist_pub_->publish(twist_msg);
  }
}

void Joy2TwistNode::call_trigger_service(const rclcpp::Client<SrvTrigger>::SharedPtr & client) const
{
  if (!client->wait_for_service(std::chrono::milliseconds(2000))) {
    RCLCPP_ERROR(this->get_logger(), "Can't contact %s service", client->get_service_name());
    return;
  }

  const auto request = std::make_shared<SrvTrigger::Request>();

  client->async_send_request(request, [&](const rclcpp::Client<SrvTrigger>::SharedFuture future) {
    trigger_service_cb(future, client->get_service_name());
  });
}

void Joy2TwistNode::trigger_service_cb(
  const rclcpp::Client<SrvTrigger>::SharedFuture & future, const std::string & service_name) const
{
  if (!future.get()->success) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to call %s service: %s", service_name.c_str(),
      future.get()->message.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully called %s service", service_name.c_str());
}

void Joy2TwistNode::handle_e_stop(const std::shared_ptr<MsgJoy> joy_msg)
{
  if (!e_stop_present_) {
    return;
  }

  if (get_joy_input_as_btn(joy_msg, input_index_.e_stop_trigger)) {
    if (!e_stop_state_) {
      // Stop the robot before trying to call the e-stop trigger service
      publish_twist(MsgTwist());
      call_trigger_service(e_stop_trigger_client_);
    }
    return;
  }

  if (
    get_joy_input_as_btn(joy_msg, input_index_.enable_e_stop_reset) &&
    get_joy_input_as_btn(joy_msg, input_index_.e_stop_reset) && e_stop_state_) {
    call_trigger_service(e_stop_reset_client_);
  }
}

void Joy2TwistNode::convert_joy_to_ackermann(
  const MsgJoy::SharedPtr joy_msg, MsgAckermannDrive & ackermann_msg)
{
  float linear_velocity_factor{}, angular_velocity_factor{};
  std::tie(linear_velocity_factor, angular_velocity_factor) = determine_velocity_factor(joy_msg);

  // Get linear velocity (speed)
  float speed = linear_velocity_factor * get_joy_input(joy_msg, input_index_.linear_x);
  ackermann_msg.speed = speed;

  // Convert angular velocity to steering angle using Ackermann geometry
  // steering_angle = atan(angular_velocity * wheelbase / linear_velocity)
  // For low speeds or zero velocity, use angular input directly scaled
  float angular_z = angular_velocity_factor * get_joy_input(joy_msg, input_index_.angular_z);
  
  if (std::abs(speed) > 0.01) {
    // Calculate steering angle from desired angular velocity
    ackermann_msg.steering_angle = std::atan(angular_z * wheelbase_ / speed);
  } else {
    // At low/zero speed, scale angular input to steering angle
    // Assuming max angular velocity of 1.0 rad/s corresponds to max steering angle
    ackermann_msg.steering_angle = angular_z * 0.5;  // Scale factor for direct mapping
  }

  // Set acceleration and other fields to zero (can be configured later)
  ackermann_msg.steering_angle_velocity = 0.0;
  ackermann_msg.acceleration = 0.0;
  ackermann_msg.jerk = 0.0;
}

void Joy2TwistNode::publish_ackermann(const MsgAckermannDrive & ackermann_msg)
{
  if (ackermann_stamped_) {
    MsgAckermannDriveStamped ackermann_stamped_msg;
    ackermann_stamped_msg.header.stamp = this->get_clock()->now();
    ackermann_stamped_msg.header.frame_id = "base_link";
    ackermann_stamped_msg.drive = ackermann_msg;
    ackermann_stamped_pub_->publish(ackermann_stamped_msg);
  } else {
    ackermann_pub_->publish(ackermann_msg);
  }
}

}  // namespace joy2twist
