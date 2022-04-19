#include "KinovaControl.h"

#include <chrono>
#include <sstream>

#include "KinovaRobot.h"
#include "impedance_controller.h"
#include "RobotMath.h"

KinovaControl::KinovaControl(std::string name_,
                             std::shared_ptr<KinovaRobot> robot_,
                             rclcpp::Node::SharedPtr node_)
    : name(name_), robot(robot_), node(node_) 
  {
  std::stringstream s;
  s << name_ << "_impedance_controller";
  impedanceController = std::make_shared<ImpedanceController>(s.str());

  node->declare_parameter<bool>(name + "/gravity_compensation", true);
  node->declare_parameter<bool>(name + "/coriolis_compensation", true);
  node->declare_parameter<bool>(name + "/pause", false);
  using namespace std::chrono;

  timer_ =
      node->create_wall_timer(200ms, std::bind(&KinovaControl::respond, this));
  gravity_compensation = false;
  coriolis_compensation = false;
  pause = false;
}

KinovaControl::~KinovaControl() {}

void KinovaControl::respond() {
  bool pause_, gravity_compensation_, coriolis_compensation_;
  pause_ = false;
  this->node->get_parameter(name + "/gravity_compensation",
                            gravity_compensation_);
  this->node->get_parameter(name + "/coriolis_compensation",
                            coriolis_compensation_);
  this->node->get_parameter(name + "/pause",
                            pause_);
  gravity_compensation = gravity_compensation_;
  coriolis_compensation = coriolis_compensation_;
  if (pause_ != pause && pause_) {
    setPause();
  }
  pause = pause_;
}

void KinovaControl::setPause() {
  std::lock_guard<std::mutex> lock(pauseLock);
  pausePose = this->robot->getCartesianPoseOrientation();
}
Eigen6f KinovaControl::getPause() {
  std::lock_guard<std::mutex> lock(pauseLock);
  return pausePose;
}

Eigen7f KinovaControl::controller(Eigen6f position) {
  // auto& clk = *this->node->get_clock();
  // RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("Runner"), clk, 100,
  // "demand: " << position << " -- actual: "<< this->robot->get_pose() );
  auto u = impedanceController->calculate(this->robot->get_pose(), 
                                          this->robot->get_velocity(), 
                                          position);
  return u;
}

Eigen7f KinovaControl::compensation(Eigen7f u) {

  return Eigen7f::Zero();
}
