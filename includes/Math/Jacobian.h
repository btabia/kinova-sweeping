/**
 * @file MassMatrix.h
 * @author Ozan Tokatli
 * @brief 
 * @version 0.1
 * @date 2021-12-19
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include "Utils.h"

/**
 * @brief Autogenerated function which calculates mass matrix for a position.
 *
 * @param q The joint angles of the robot (degrees)
 * @param M The kinematic Jacobian matrix expressed in the base frame
 */
void jacobian(const Eigen::Matrix<float, 7, 1> q,
              Eigen::Matrix<float, 6, 7> &J);