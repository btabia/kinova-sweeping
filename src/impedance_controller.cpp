/**
 * @file impedance_controller.cpp
 * @author Bechir Tabia
 * @brief 
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "impedance_controller.h"
#include "Math/Jacobian.h"
#include "Math/Coriolis.h"
#include "Math/ForwardKinematics.h"
#include "Math/Gravity.h"
#include <atomic>
#include <chrono>
#include <functional>

ImpedanceController::ImpedanceController(std::string name) : Node(name), name(name) {
  std::map<std::string, float> params;

  float linear_stiffness = 1500; 
  float linear_damping = 0.1;

  float rotational_stiffness = 1500;
  float rotational_damping = 0.1; 


  stiffness << linear_stiffness, linear_stiffness, linear_stiffness, rotational_stiffness, rotational_stiffness, rotational_stiffness;
  damping << linear_damping, linear_damping, linear_damping, rotational_damping, rotational_damping, rotational_damping;

  for (int i = 1; i < 6; ++i) {
    params["/axis_" + std::to_string(i) + "/stiffness"] = stiffness[i-1];
    params["/axis_" + std::to_string(i) + "/damping"] = damping[i-1];
  }
  this->declare_parameters(name, params);

}

void ImpedanceController::respond() {
  std::map<std::string, float> params;
  this->get_parameters(name, params);
  Eigen6f stiffness_, damping_;
  for (int i = 0; i < 6; ++i) {
    stiffness_[i] = params["/axis_" + std::to_string(i+1) + "/stiffness"];
    damping_[i] = params["/axis_" + std::to_string(i+1) + "/damping"];
  }
  setStiffness(stiffness_);
  setDamping(damping_);
}

Eigen6f ImpedanceController::getStiffness() {
  std::lock_guard<std::mutex> lock(lock_stiffness);
  return stiffness;
}

Eigen6f ImpedanceController::getDamping() {
  std::lock_guard<std::mutex> lock(lock_damping);
  return damping;
}

void ImpedanceController::setStiffness(Eigen6f _stiffness) {
  std::lock_guard<std::mutex> lock(lock_stiffness);
  stiffness = std::forward<Eigen6f>(_stiffness);
}

void ImpedanceController::setDamping(Eigen6f _damping) {
  std::lock_guard<std::mutex> lock(lock_damping);
  damping = std::forward<Eigen6f>(_damping);
}

float loop(float x) {
  if (x < -180.0)
    return fmod(x + 360.0f, 360.0f);
  if (x > 180.0)
    return fmod(x - 360.0f, 360.0f);
  return x;
}

float loopRad(float x) {
  if (x < -(M_PI_2))
    return fmod(x + M_PI, M_PI);
  if (x > 180.0)
    return fmod(x - M_PI, M_PI);
  return x;
}

Eigen7f ImpedanceController::calculate(Eigen7f pActual, Eigen7f dqActual, Eigen6f pDemand) 
{
  // intermediary data
  Eigen::Matrix<float, 3,1> pose_observation = Eigen3f::Zero();
  Eigen::Matrix<float,3,1> pose_command = pDemand.head(3);
  Eigen::Matrix<float,3,3> orientation = Eigen3x3f::Zero();
  Eigen::Matrix<float,6,7> jacobian_matrix;
  Eigen7f coriolis_force = Eigen7f::Zero();



  forward_kinematics(pActual, pose_observation, orientation);
  jacobian(pActual, jacobian_matrix);
  coriolis(pActual, dqActual, coriolis_force);

  Eigen3f oActual = orientation.eulerAngles(0,1,2);

  Eigen6f actual;
  actual.head(3) = pose_observation;
  actual.tail(3) = oActual;

  std::cout << "Demand: \n" << pDemand << std::endl;
  std::cout << "Actual: \n" << actual << std::endl;


  // DEMAND
  Eigen::Quaternionf orientation_command;
  orientation_command =  Eigen::AngleAxisf(pDemand(3) , Eigen::Vector3f::UnitX())
                    *  Eigen::AngleAxisf(pDemand(4) , Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(pDemand(5) , Eigen::Vector3f::UnitZ());
  // OBSERVATION
  Eigen::Quaternionf orientation_observation(orientation);

  // Compute position error
  Eigen::Matrix<float, 6,1> error;
  error.head(3) <<  (pose_observation - pose_command);
  // Compute orientation error
  // Difference quaternion
  if (orientation_command.coeffs().dot(orientation_observation.coeffs()) < 0.0) {
  orientation_observation.coeffs() << -orientation_observation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaternionf error_quaternion(orientation_observation.inverse() * orientation_command);
  //error.tail(3) << error_quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -orientation * error.tail(3);

  std::cout << "Error: \n" << error << std::endl;

  //compute control
  Eigen::Matrix<float, 7,1> tau;
  Eigen::Matrix<float, 7,1> tau_output;

  Eigen::Matrix<float, 6,6> stiffness_matrix; 
  Eigen::Matrix<float, 6,6> damping_matrix;

  stiffness_matrix << stiffness(0),0,0,0,0,0,
                      0,stiffness(1),0,0,0,0,
                      0,0,stiffness(2),0,0,0,
                      0,0,0,stiffness(3),0,0,
                      0,0,0,0,stiffness(4),0,
                      0,0,0,0,0,stiffness(5);

  damping_matrix << damping(0),0,0,0,0,0,
                    0,damping(1),0,0,0,0,
                    0,0,damping(2),0,0,0,
                    0,0,0,damping(3),0,0,
                    0,0,0,0,damping(4),0,
                    0,0,0,0,0,damping(5);

  tau << jacobian_matrix.transpose() * (-stiffness_matrix * error - damping_matrix * (jacobian_matrix * dqActual));
  tau_output << tau + coriolis_force;

  // large actuator torque limit
  for(unsigned int i = 0; i < 4; i++)
  {
    if(tau_output(i) > 105) tau_output(i) = 105;
    if(tau_output(i) < -105) tau_output(i) = -105;
  }
  // small actuator torque limit
  for(unsigned int i = 4; i < 7; i++)
  {
    if(tau_output(i) > 52) tau_output(i) = 52;
    if(tau_output(i) < -52) tau_output(i) = -52;
  }
  std::cout << "Torque output: \n" << tau_output << std::endl;
  return tau_output;
}

