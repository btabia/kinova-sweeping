#pragma once

/// \cond
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <atomic>
#include <std_srvs/srv/trigger.hpp>
/// \endcond

using std::placeholders::_1;

class BilateralNode : public rclcpp::Node {
public:
  BilateralNode();
  virtual ~BilateralNode() {}

  enum BilateralState {
    uninitialised,
    ready_to_home_local,
    homing_local,
    ready_to_home_remote,
    homing_remote,
    ready_to_start,
    running,
    stopped
  };

  std::string getBilateralState();

  using atomic_bool_ptr = std::shared_ptr<std::atomic<bool>>;
  atomic_bool_ptr home_local;
  atomic_bool_ptr home_remote;
  atomic_bool_ptr robot_start;
  atomic_bool_ptr robot_stop;

  std::atomic_bool enabled;
  std::atomic_bool proceed;

  std::atomic_bool gripper_open;
  std::atomic_bool gripper_close;

  std::atomic<BilateralState> bilateral_state;

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_home_local;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_home_remote;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_start;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_stop;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr home_local_ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr home_remote_ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_ready_publisher_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr proceed_subscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_open_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_close_subscription_;

private:
  void timer_callback();
  void check_enabled();
  void check_proceed(atomic_bool_ptr ptr);


  void enable_callback(const std_msgs::msg::Bool::SharedPtr msg) ;
  void proceed_callback(const std_msgs::msg::Bool::SharedPtr msg) ;

  void gripper_open_callback(const std_msgs::msg::Bool::SharedPtr msg) ;
  void gripper_close_callback(const std_msgs::msg::Bool::SharedPtr msg) ;


  void home_local_trigger(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    *home_local = true;
  }
  void home_remote_trigger(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    *home_remote = true;
  }
  void robot_start_trigger(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    *robot_start = true;
  }
  void robot_stop_trigger(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    *robot_stop = true;
  }
};