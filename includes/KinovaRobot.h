/**
 * @file KinovaRobot.h
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
#include <chrono>
#include <mutex>
#include <string>
#include <vector>
/// \endcond 

#include "Utils.h"

#include <math.h>
#include <time.h>

#include <functional>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <KDetailedException.h>

#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <DeviceManagerClientRpc.h>
#include <RouterClient.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <google/protobuf/util/json_util.h>

#include "Math/ForwardKinematics.h"
#include "Math/Jacobian.h"
#include "Math/Coriolis.h"



namespace Kinova {
namespace Api {
class TransportClientTcp;
class TransportClientUdp;
class RouterClient;
namespace Session {
class CreateSessionInfo;
}
class SessionManager;
namespace Base {
class BaseClient;
}
namespace BaseCyclic {
class BaseCyclicClient;
//class Feedback;/
//class Command;
} // namespace BaseCyclic
namespace GripperCyclic {
class MotorCommand;
}
namespace ActuatorConfig {
class ActuatorConfigClient;
}
} // namespace Api
} // namespace Kinova

using namespace Kinova::Api;
/**
 * @brief Kinova robot hardware interface
 *
 */
class KinovaRobot {
public:
  /**
   * @brief Construct a new Kinova Robot object
   *
   * @param name Name of the robot
   * @param address Ip address of the robot
   * @param username username of the robot
   * @param password password of the robot
   */
  KinovaRobot(std::string name, std::string address,
              std::string username = "admin", std::string password = "admin");
  /**
   * @brief Destroy the Kinova Robot object
   *
   */
  ~KinovaRobot();
  /**
   * @brief Will attempt to move the kinova to CMS_HOME
   *
   * @return true Robot homed successfully
   * @return false robot failed to home
   */
  bool home();

  enum GripperAction{ Stop, Close, Open };
  
  /**
   * @brief torque mode set up
   * 
   */
  void torqueMode();
  /**
   * @brief get observation data from robot
   * 
   * @param base_feedback 
   */
  void get(const Kinova::Api::BaseCyclic::Feedback *base_feedback);
  /**
   * @brief set command data to robot
   * 
   * @param command 
   * @param gripper 
   */
  void set(Eigen7f command, GripperAction gripper = GripperAction::Stop );
  /**
   * @brief Get the current pose  (thread safe)
   *
   * @return Eigen7f current pose (degrees)
   */
  Eigen7f get_pose() {
    std::lock_guard<std::mutex> lk(mtx_pose);
    return cached_pose;
  };
  /**
   * @brief Get the current velocity (thread safe)
   *
   * @return Eigen7f current velocity (degrees/s)
   */
  Eigen7f get_velocity() {
    std::lock_guard<std::mutex> lk(mtx_velocity);
    return cached_velocity;
  };
  /**
   * @brief Get the current torque (thread safe)
   *
   * @return Eigen7f current torque (Nm)
   */
  Eigen7f get_torque() {
    std::lock_guard<std::mutex> lk(mtx_torque);
    return cached_torque;
  };
  /**
   * @brief Get the gripper position (thread-safe)
   * 
   * @return Gripper position
   */
  float get_gripper() {
    std::lock_guard<std::mutex> lk(mtx_gripper);
    return gripper_position;
  };
  /**
   * @brief Get the command
   *
   * @return Eigen7f current command (Nm)
   */
  Eigen7f get_command() {
    std::lock_guard<std::mutex> lk(mtx_command);
    return cached_command;
  };
  /**
   * @brief Get the Cartesian Pose Orientation object
   * 
   * @return Eigen6f 
   */
  Eigen6f getCartesianPoseOrientation()
  {
    std::lock_guard<std::mutex> lk(mtx_cpose);
    return cached_cPose;
  }
  /**
   * @brief Get the Cartesian Velocity object
   * 
   * @return Eigen6f 
   */
  Eigen6f getCartesianVelocity()
  {
    std::lock_guard<std::mutex> lk(mtx_cvel);
    return cached_cVelocity;
  }
  /**
   * @brief Get the Cartesian Force Moment object
   * 
   * @return Eigen6f 
   */
  Eigen6f getCartesianForceMoment()
  {
    std::lock_guard<std::mutex> lk(mtx_cforce);
    return cached_cForce;
  }

  Eigen::Matrix3f getRotationMatrix()
  {
    std::lock_guard<std::mutex> lk(mtx_rotation_matrix);
    return cached_rotation_matrix;
  }

  Eigen::Matrix<float,6,7> getJacobianMatrix()
  {
    std::lock_guard<std::mutex> lk(mtx_jacobian);
    return cached_jacobian;
  }

  Eigen7f getCoriolis()
  {
    std::lock_guard<std::mutex> lk(mtx_coriolis);
    return cached_coriolis;
  }

  Eigen7f getTorqueOffset()
  {
    std::lock_guard<std::mutex> lk(mtx_torque_offset);
    return cached_torque_offset;
  }

  Eigen7f getCommand()
  {
    std::lock_guard<std::mutex> lk(mtx_torque_offset);
    return cached_command;
  }

  Eigen7f getFirstPosition()
  {
    std::lock_guard<std::mutex> lk(mtx_first_position);
    return first_position;
  }
  /**
   * @brief Set the command
   *
   * @param actual actual command (torque)
   */
  void setCommand(Eigen7f actual) {
    std::lock_guard<std::mutex> lk(mtx_command);
    cached_command = actual;
  }
  /**
   * @brief Set robot into position high level control mode
   *
   */
  void positionMode();
  /**
   * @brief Attempt to clear any faults of the robot
   *
   * @return true Succesfully cleared faults if there were any
   * @return false Didnt clear faults, robot won't continue
   */
  bool clearFault();
  /**
   * @brief Get the All connected Devices
   *
   */
  void getAllDevices();

  void setState(std::string state_){
    std::lock_guard<std::mutex> lk(mtx_state);
    state = state_;
  }

  std::string getState(){
    std::lock_guard<std::mutex> lk(mtx_state);
    return state;
  }

private: 
  void setupGripper();
  void openGripper();
  void closeGripper();
  void stopGripper();
  void doGripperAction(GripperAction gripperAction);
  void getGripper(const Kinova::Api::BaseCyclic::Feedback *base_feedback);


public:
  void set_pose(Eigen7f actual) {
    std::lock_guard<std::mutex> lk(mtx_pose);
    cached_pose = actual;
  }
  void set_velocity(Eigen7f actual) {
    std::lock_guard<std::mutex> lk(mtx_velocity);
    cached_velocity = actual;
  }
  void set_torque(Eigen7f actual) {
    std::lock_guard<std::mutex> lk(mtx_torque);
    cached_torque = actual;
  }

  void set_gripper(float actual) {
    std::lock_guard<std::mutex> lk(mtx_gripper);
    gripper_position = actual;
  }

  void setCartesianPoseOrientation(Eigen6f actual)
  {
      std::lock_guard<std::mutex> lk(mtx_cpose);
      cached_cPose = actual;
  }

  void setCartesianVelocity(Eigen6f actual)
  {
      std::lock_guard<std::mutex> lk(mtx_cvel);
      cached_cVelocity = actual;
  }

  void setCartesianForceMoment(Eigen6f actual)
  {
      std::lock_guard<std::mutex> lk(mtx_cforce);
      cached_cForce = actual;
  }

  void setRotationMatrix(Eigen::Matrix3f actual)
  {
    std::lock_guard<std::mutex> lk(mtx_rotation_matrix);
    cached_rotation_matrix = actual;
  }

  void setJacobianMatrix(Eigen::Matrix<float, 6,7> actual)
  {
    std::lock_guard<std::mutex> lk(mtx_jacobian);
    cached_jacobian = actual;
  }

  void setCoriolis(Eigen7f actual)
  {
    std::lock_guard<std::mutex> lk(mtx_coriolis);
    cached_coriolis = actual;
  }

  void setTorqueOffset(Eigen7f actual)
  {
    std::lock_guard<std::mutex> lk(mtx_torque_offset);
    cached_torque_offset = actual;
  }

  void setFirstPosition(Eigen7f actual)
  {
    std::lock_guard<std::mutex> lk(mtx_first_position);
    first_position = actual;
  }
  
public:
  /**
   * @brief Name of the robot.
   *
   */
  const std::string name;
  /**
   * @brief Number of joints in the robot.
   * 
   */
  static constexpr unsigned int actuator_count = 7;

  bool has_gripper;

private:
  // API objects
  TransportClientTcp *transport;
  RouterClient *router;

  TransportClientUdp *transport_real_time;
  RouterClient *router_real_time;

  // session data connection information
  Session::CreateSessionInfo *create_session_info;

  // Session manager service wrapper
  SessionManager *session_manager;
  SessionManager *session_manager_real_time;

  // services
  Base::BaseClient *base;
  BaseCyclic::BaseCyclicClient *base_cyclic;
  
  // BaseCyclic::Feedback *base_feedb;
  ActuatorConfig::ActuatorConfigClient *actuator_config;
 
  Kinova::Api::BaseCyclic::Feedback base_feedback;
  Kinova::Api::BaseCyclic::Command *base_command;
  GripperCyclic::MotorCommand*   gripper_motor_command;

  // Cached
  // mutexes
  std::mutex mtx_pose, mtx_velocity, mtx_torque, mtx_command, mtx_state;
  std::mutex mtx_cpose, mtx_cvel, mtx_cforce, mtx_rotation_matrix, mtx_jacobian;
  std::mutex mtx_coriolis;
  std::mutex mtx_torque_offset; 
  std::mutex mtx_first_position;

  //data
  Eigen7f cached_pose, cached_velocity, cached_torque, cached_command, cached_coriolis;
  Eigen6f cached_cPose, cached_cVelocity, cached_cForce;
  Eigen::Matrix3f cached_rotation_matrix;
  Eigen::Matrix<float,6,7> cached_jacobian;
  Eigen7f cached_torque_offset; 
  Eigen7f first_position;

  // Gripper
  std::mutex mtx_gripper;
  float gripper_position; 

  // Timing variables
  std::chrono::high_resolution_clock::time_point last_time;
  std::vector<double> times;
  unsigned int step;
  const static unsigned int averaging_window = 500;

  // Mocking variables
  bool first = true;
  Eigen7f torque_actual, velocity_actual, position_actual;
  Eigen7f torque_last, velocity_last, position_last;

  // State
  std::string state;
};
