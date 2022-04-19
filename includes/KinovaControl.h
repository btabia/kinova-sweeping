/**
 * @file KinovaControl.h
 * @author Guy Burroughes
 * @brief 
 * @version 0.1
 * @date 2021-12-19
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once
/// \cond 
#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
/// \endcond 

#include "KinovaRobot.h"
#include "impedance_controller.h"
#include "Utils.h"

/**
 * @brief Robot Controllersand lineariser object
 * Tunable via ROS params.
 * 
 * ROS parameters:
 * 
 *  - Gravity Compensation - Turns on Gravity Compesation
 *  - Coriolis Compensation - Turns on Gravity Compesation
 *  - Pause - Turns on a hold position controller
 *
 */
class KinovaControl {
public:
  /**
   * @brief Construct a new Kinova Control object
   *
   * @param name Name of the robot
   * @param robot Robot hardware interface
   * @param node ros node for logging
   */
  KinovaControl(std::string name, std::shared_ptr<KinovaRobot> robot,
                rclcpp::Node::SharedPtr node);

  /**
   * @brief Destroy the Kinova Control object
   *
   */
  ~KinovaControl();

  

  /**
   * @brief Lineraise control input (Inertia, Coroilis, gravity)
   *
   * @param u control input
   * @return Eigen7f linearised control output
   */
  Eigen7f compensation(Eigen7f u);

    /**
   * @brief Calculate control to set position and velocity (wraps PD currently)
   *
   * @param position Desired Position
   * @param velocity Desired velocity
   * @return Eigen6f Control output
   */
  Eigen7f controller(Eigen6f position);
/**
   * @brief Get the Impedance Controller object
   * 
   * @return ImpedanceController::SharedPtr 
   */
  ImpedanceController::SharedPtr getImpedanceController() {return impedanceController;}

private:
  /**
   * @brief ROS param updater
   *
   */
  void respond();

  void setPause();
  Eigen6f getPause();

public:
  /**
   * @brief Name of the robot
   *
   */
  const std::string name;
  /**
   * @brief Robot hardware interface
   * 
   */
  std::shared_ptr<KinovaRobot> robot;

private:
  // Controller
  std::shared_ptr<ImpedanceController> impedanceController;
  rclcpp::Node::SharedPtr node;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> gravity_compensation, coriolis_compensation, pause;

  std::mutex pauseLock;
  Eigen6f pausePose;

  Eigen7f integrated_torque;
};
