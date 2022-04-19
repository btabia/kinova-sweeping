/**
 * @file TaskPublisher.h
 * @author Bechir Tabia
 * @brief Publish the task 
 * @version 0.1
 * @date 2022-03-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

/// \cond
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
/// \endcond


#include "KinovaRobot.h"


using namespace std::chrono_literals;

/**
 * @brief A ROS2 node that runs in a executor thread to allow ros logging
 * without limiting 1k loop.
 *
 */
class TaskPublisher : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Task Publisher object
   *
   * @param k_ Robot hardware interface
   */
  explicit TaskPublisher(std::shared_ptr<KinovaRobot> k_);

private:
  /**
   * @brief The timed callback which publishes the robot state.
   *
   */
  void timer_callback();

private:
  /**
   * @brief Timer for ROS publisher
   *
   */
  rclcpp::TimerBase::SharedPtr timer_;
  /**
   * @brief Robot command message to send
   * 
   */
  sensor_msgs::msg::Twist task_state;
    /**
     * @brief 
     * 
     */
  std_msgs::msg::String state_message;
  /**
   * @brief Robot Twist message publisher
   * 
   */
  rclcpp::Publisher<sensor_msgs::msg::Twist>::SharedPtr robot_publisher_;
  /**
   * @brief Command message publisher
   * 
   */
  rclcpp::Publisher<sensor_msgs::msg::Twist>::SharedPtr planning_command_publisher_;

  /**
   * @brief Shared pointer to robot hardware interface to get cached pose, velocity and torque.
   * 
   */
  std::shared_ptr<KinovaRobot> k;

  std::shared_ptr</*micro planned*/> planner;
};