#include "BilateralNode.h"

BilateralNode::BilateralNode() : Node("kinova_bilateral") {

  using std::placeholders::_1;
  using std::placeholders::_2;

  home_local = std::make_shared<std::atomic<bool>>();
  home_remote = std::make_shared<std::atomic<bool>>();
  robot_start = std::make_shared<std::atomic<bool>>();
  robot_stop = std::make_shared<std::atomic<bool>>();

  service_home_local = this->create_service<std_srvs::srv::Trigger>(
      "home_robot_local",
      std::bind(&BilateralNode::home_local_trigger, this, _1, _2));
  service_home_remote = this->create_service<std_srvs::srv::Trigger>(
      "home_robot_remote",
      std::bind(&BilateralNode::home_remote_trigger, this, _1, _2));

  service_start = this->create_service<std_srvs::srv::Trigger>(
      "robot_start",
      std::bind(&BilateralNode::robot_start_trigger, this, _1, _2));

  service_stop = this->create_service<std_srvs::srv::Trigger>(
      "robot_stop",
      std::bind(&BilateralNode::robot_stop_trigger, this, _1, _2));

  publisher_ =
      this->create_publisher<std_msgs::msg::String>("bilateral_state", 10);
  home_local_ready_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      "bilateral/home_local_ready", 10);
  home_remote_ready_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      "bilateral/home_remote_ready", 10);
  start_ready_publisher_ =
      this->create_publisher<std_msgs::msg::Bool>("bilateral/start_ready", 10);
  stop_ready_publisher_ =
      this->create_publisher<std_msgs::msg::Bool>("bilateral/stop_ready", 10);

  enable_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "enable", 10, std::bind(&BilateralNode::enable_callback, this, _1));

  proceed_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "proceed", 10, std::bind(&BilateralNode::proceed_callback, this, _1));

  gripper_open_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "gripper_open", 10, std::bind(&BilateralNode::gripper_open_callback, this, _1));
  gripper_close_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "gripper_close", 10, std::bind(&BilateralNode::gripper_close_callback, this, _1));

  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(
      200ms, std::bind(&BilateralNode::timer_callback, this));
}

std::string BilateralNode::getBilateralState() {

  const std::map<BilateralNode::BilateralState, const char *>
      BilateralStateStrings{
          {BilateralNode::BilateralState::uninitialised, "Initialising"},
          {BilateralNode::BilateralState::ready_to_home_local,
           "Ready to home local"},
          {BilateralNode::BilateralState::homing_local, "Homing local robot"},
          {BilateralNode::BilateralState::homing_remote, "Homing remote robot"},

          {BilateralNode::BilateralState::ready_to_home_remote,
           "Ready to home remote robot"},
          {BilateralNode::BilateralState::ready_to_start, "Ready to start"},
          {BilateralNode::BilateralState::running, "Running"},
          {BilateralNode::BilateralState::stopped, "Stopped"}};
  auto it = BilateralStateStrings.find(bilateral_state);
  return it == BilateralStateStrings.end() ? "Out of range" : it->second;
}

void BilateralNode::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = getBilateralState();
  publisher_->publish(message);
  auto ready_home_local = std_msgs::msg::Bool();
  ready_home_local.data = false;
  auto ready_home_remote = std_msgs::msg::Bool();
  ready_home_remote.data = false;
  auto ready_to_start_msg = std_msgs::msg::Bool();
  ready_to_start_msg.data = false;
  auto ready_to_stop_msg = std_msgs::msg::Bool();
  ready_to_stop_msg.data = false;
  switch (bilateral_state) {
  case ready_to_home_local:
    ready_home_local.data = true;
    check_proceed(home_local);
    break;
  case ready_to_home_remote:
    ready_home_remote.data = true;
    check_proceed(home_remote);
    break;
  case ready_to_start:
    ready_to_start_msg.data = true;
    check_proceed(robot_start);
    break;
  case BilateralState::running:
    ready_to_stop_msg.data = true;
    check_enabled();
    break;
  default:
    break;
  }
  home_local_ready_publisher_->publish(ready_home_local);
  home_remote_ready_publisher_->publish(ready_home_remote);
  start_ready_publisher_->publish(ready_to_start_msg);
  stop_ready_publisher_->publish(ready_to_stop_msg);
}

void BilateralNode::check_enabled() {
  if (!enabled) {
    RCLCPP_WARN(this->get_logger(), "Enable swtich released! Stopping robots.");
    *robot_stop = true;
  }
  enabled = false; // reset the flag.
}

void BilateralNode::check_proceed(std::shared_ptr<std::atomic<bool> > ptr) {
  if (proceed) {
    RCLCPP_INFO(this->get_logger(), "Proceed button pressed!");
    *ptr = true;
  }
  proceed = false; // reset the flag.
}

void BilateralNode::enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  enabled = msg->data;
}

void BilateralNode::proceed_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  proceed = msg->data;
}


void BilateralNode::gripper_open_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  gripper_open = msg->data;
}

void BilateralNode::gripper_close_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  gripper_close = msg->data;
}
