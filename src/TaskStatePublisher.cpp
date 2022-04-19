#include "TaskPublisher.h"

TaskPublisher::TaskPublisher(std::shared_ptr<KinovaRobot> k_)
    : rclcpp::Node("Task_State_Publisher"), k(k_) {
  std::stringstream topic_name;
  topic_name << "/" << k->name << "/task_state";

  robot_publisher_ = this->create_publisher<sensor_msgs::msg::Twist>(
      topic_name.str(), 10);
  timer_ = this->create_wall_timer(
      50ms, std::bind(&TaskPublisher::timer_callback, this));
  for (unsigned int i = 1; i <= KinovaRobot::actuator_count; ++i) {
    std::stringstream ss;
    ss << "Axis" << i;
    robot_message.name.push_back(ss.str());
    command_message.name.push_back(ss.str());
  }
}

void TaskPublisher::timer_callback() {
  Eigen6f task_pose = k->getCartesianPoseOrientation();
  Eigen6f task_vel = k->getCartesianVelocity();
  Eigen7f actual_pose = k->get();
  Eigen7f actual_velocity = k->get_velocity();
  Eigen7f actual_torque = k->get_torque();
  Eigen7f command = k->get_command();
  robot_message.position.clear();
  robot_message.effort.clear();
  robot_message.velocity.clear();
  command_message.effort.clear();
  for (int i = 0; i < 7; ++i) {
    robot_message.position.push_back(actual_pose[i]);
    robot_message.velocity.push_back(actual_velocity[i]);
    robot_message.effort.push_back(actual_torque[i]);
    command_message.effort.push_back(command[i]);
  }
  state_message.data = k->getState();

  float gripper = k->get_gripper();
  gripper_message.data = gripper;

  robot_publisher_->publish(robot_message);
  command_publisher_->publish(command_message);
  state_publisher_->publish(state_message);
  gripper_publisher_->publish(gripper_message);
}