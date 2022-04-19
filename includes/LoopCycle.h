/**
 * @file LoopCycle.h
 * @author Guy Burroughes
 * @brief
 * @version 0.1
 * @date 2022-01-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once
/// \cond
#include <KDetailedException.h>
#include <chrono>
#include <iostream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>
#include <string>
#include <thread>
/// \endcond

#include "JointStatePublisher.h"
#include "KinovaControl.h"
#include "KinovaRobot.h"
#include "planning/GeneralPlanner.h"
#include "Math/ForwardKinematics.h"

/**
 * @brief The class that runs the 1KHz control loop.
 * File has 2 separate versions, one for bilateral and one for unilateral.
 * This is too avoid function pointers in the main loop cycle.
 *
 */
class LoopCycle {
public:
  /**
   * @brief Main control loop for unilateral
   *
   * @param node Ros node for logging
   * @param robot Robot hardware interface
   * @param robotControl Controllers interface
   */
  explicit LoopCycle(rclcpp::Node::SharedPtr node,
                     std::shared_ptr<KinovaRobot> robot,
                     std::shared_ptr<KinovaControl> robotControl,
                     std::shared_ptr<GeneralPlanner> general_planner);

  LoopCycle(const LoopCycle &loopCycle);
  LoopCycle(LoopCycle &&loopCycle);

  /**
   * @brief data initialisation before running the main 1Khz loop
   *
   */
  void init();

  /**
   * @brief the Main 1KHz loop
   *
   */
  void loop();

  /**
   * @brief Set the Position demand
   *
   * @param pose demand pose
   * @param velocity demand velocity
   */
  void setDemand(Eigen7f pose, Eigen7f velocity);

  /**
   * @brief Set the task demand
   *
   * @param pose demand pose (x, y, z, r, p, yaw)
   */
  void setTaskDemand(Eigen6f pose);
  /**
   * @brief Set the Sweeping Demand object
   * 
   * @param pose 
   */
  void setSweepingDemand(Eigen6f pose);
  /**
   * @brief Set the Starting Pose object
   * 
   * @param pose 
   */
  void setStartingPose(Eigen6f pose);


private:
  /**
   * @brief Control function run in loop()
   *
   */
  void control();

  /**
   * @brief Put robots into torque mode
   *
   */
  void torqueMode();
  /**
   * @brief Put robots into position mode
   *
   */
  void positionMode();

  std::pair<Eigen7f, Eigen7f> getDemand();
  /**
   * @brief Get the Task Demand object
   * 
   * @return Eigen6f 
   */
  Eigen6f getTaskDemand();
  /**
   * @brief Get the Sweeping Demand object
   * 
   * @return Eigen6f 
   */
  Eigen6f getSweepingDemand();
  /**
   * @brief Get the Starting Pose object
   * 
   * @return Eigen6f 
   */
  Eigen6f getStartingPose();

private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<KinovaRobot> local;
  std::shared_ptr<KinovaControl> localControl;

  Eigen7f demandPose, demandVelocity;
  Eigen6f demandTask;
  Eigen6f demandSweeping;
  Eigen6f starting_pose;
  std::mutex demandMutex;
  std::mutex demandTaskMutex;
  std::mutex demandSweepingMutex;
  std::mutex mtx_starting_pose; 

  /**
   * @brief general planning generator
   * 
   */
  std::shared_ptr<GeneralPlanner> planner; 
};