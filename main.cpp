/**
 * @file main.cpp
 * @author Guy Burroughes
 * @brief The main file for the Bilateral executable
 * @version 0.1
 * @date 2022-01-09
 *
 * @copyright Copyright (c) 2022
 *
 */

/*! \file */

/// \cond
#include <KDetailedException.h>
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

/// \endcond

#include "BilateralNode.h"
#include "Input.h"
#include "JointStatePublisher.h"
#include "KinovaControl.h"
#include "KinovaRobot.h"
#include "LoopCycle.h"

using namespace std;

int main(int argc, char *argv[]) {
  // Set Environment variables
  set_env_var();

  // Initialise ROS2
  rclcpp::init(argc, argv);

  // Initialise variable
  auto node = std::make_shared<BilateralNode>();
  RCLCPP_INFO(rclcpp::get_logger("Main"), "Connecting to robots.");
  auto local = std::make_shared<KinovaRobot>("local", "192.168.1.12");
  auto remote = std::make_shared<KinovaRobot>("remote", "192.168.1.111");
  auto localControl = std::make_shared<KinovaControl>("local", local, node);
  auto remoteControl = std::make_shared<KinovaControl>("remote", remote, node);

  local->has_gripper = false;
  remote->has_gripper = true;

  node->bilateral_state = BilateralNode::BilateralState::uninitialised;

  rclcpp::executors::MultiThreadedExecutor executor;
  auto localPublisher = std::make_shared<JointStatePublisher>(local);
  auto remotePublisher = std::make_shared<JointStatePublisher>(remote);

  executor.add_node(localPublisher);
  executor.add_node(remotePublisher);
  executor.add_node(node);
  executor.add_node(localControl->getPdController());
  executor.add_node(remoteControl->getPdController());

  LoopCycle loopCycle(node, local, localControl, remote, remoteControl);

  // Declare ROS2 parameters
  node->declare_parameter<bool>("lateral_run", false);

  // Clear Faults
  local->clearFault();
  remote->clearFault();

  // Home the robots
  node->bilateral_state = BilateralNode::BilateralState::ready_to_home_local;
  RCLCPP_INFO(rclcpp::get_logger("Main"), "Ready to home robots.");
  RCLCPP_INFO(rclcpp::get_logger("Main"), "Home the local robot? [Y/n]");
  auto cont = spin_till_input(&executor, node->home_local);
  if (!cont)
    return 0;
  node->bilateral_state = BilateralNode::BilateralState::homing_local;
  RCLCPP_INFO(rclcpp::get_logger("Main"), "Homing local robot.");
  local->home();

  node->bilateral_state = BilateralNode::BilateralState::ready_to_home_remote;
  RCLCPP_INFO(rclcpp::get_logger("Main"), "Home the remote robot? [Y/n]");
  cont = spin_till_input(&executor, node->home_remote);
  if (!cont)
    return 0;
  node->bilateral_state = BilateralNode::BilateralState::homing_remote;
  RCLCPP_INFO(rclcpp::get_logger("Main"), "Homing remote robot.");
  remote->home();

  // Start Control
  node->bilateral_state = BilateralNode::BilateralState::ready_to_start;
  RCLCPP_INFO(rclcpp::get_logger("Main"), "Press [Y/y] to start control...");
  cont = spin_till_input(&executor, node->robot_start);
  if (!cont)
    return 0;
  RCLCPP_INFO(rclcpp::get_logger("Main"), "Starting control bilateral robot.");

  // Start loop thread
  std::thread runner(&LoopCycle::loop, loopCycle);

  // Spin ROS2 services
  // executor.spin();
  node->bilateral_state = BilateralNode::BilateralState::running;
  RCLCPP_INFO(rclcpp::get_logger("Main"), "Press enter to stop!");
  cont = spin_till_input(&executor, node->robot_stop);
  RCLCPP_INFO(rclcpp::get_logger("Main"), "Pausing");

  // set stop parameter
  rclcpp::Parameter action_param("lateral_run", true);
  node->set_parameter(action_param);
  node->bilateral_state = BilateralNode::BilateralState::stopped;

  // Stop threads and exit
  runner.join();

  rclcpp::sleep_for(1s);

  rclcpp::shutdown();
  return 0;
}