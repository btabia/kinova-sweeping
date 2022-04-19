/**
 * @file impedance_controller.h
 * @author Bechir Tabia
 * @brief 
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

/// \cond
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <string>
/// \endcond 

#include "Utils.h"
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "Math/Coriolis.h"
#include "Math/Gravity.h"
#include "Math/MassMatrix.h"
#include "Math/Jacobian.h"

/// \cond
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <string>
/// \endcond 


/**
 * @brief A PD controller that work in degrees [0,360]
 * Loops around 360 inflection point.
 * Kp and Kd are ROS params that update every 500ms.
 */
class ImpedanceController : public rclcpp::Node {
public:
  /**
   * @brief Construct a new PDController object
   *
   * @param name Name of robot under control.
   */
  explicit ImpedanceController(std::string name);

  /**
   * @brief updates stiffness and damping gain from ROS params, is running in a thread
   *
   */
  void respond();
  /**
   * @brief 
   * 
   * @param pActual 
   * @param qdActual 
   * @param pDemand 
   * @return Eigen7f 
   */
  Eigen7f calculate(Eigen7f pActual, Eigen7f dqActual, 
                    Eigen6f pDemand);

  /**
   * @brief Get the Kp object (thread safe)
   *
   * @return Eigen7f Kp
   */
  Eigen6f getStiffness();
  /**
   * @brief Get the Kd object (thread safe)
   *
   * @return Eigen7f Kd
   */
  Eigen6f getDamping();

private:
/**
 * @brief Set the Stiffness object
 * 
 * @param _stiffness 
 */
  void setStiffness(Eigen6f _stiffness);
/**
 * @brief Set the Damping object
 * 
 * @param _damping 
 */
  void setDamping(Eigen6f _damping);

private:
  /**
   * @brief Control multipliers
   *
   */
  Eigen6f stiffness, damping;
  /**
   * @brief Locks for thread safe setters an getters of Kp and Kd
   *
   */
  std::mutex lock_stiffness, lock_damping;
  /**
   * @brief Robot name
   * 
   */
  std::string name;
};
