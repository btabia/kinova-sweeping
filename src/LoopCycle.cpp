#include "LoopCycle.h"

LoopCycle::LoopCycle(rclcpp::Node::SharedPtr node,
                     std::shared_ptr<KinovaRobot> robot,
                     std::shared_ptr<KinovaControl> robotControl,
                     std::shared_ptr<GeneralPlanner> robotPlanner)
    : node(node), local(robot), localControl(robotControl), planner(robotPlanner)
    {
    }


std::pair<Eigen7f, Eigen7f> LoopCycle::getDemand() {
  std::lock_guard<std::mutex> lock(demandMutex);
  return std::make_pair(demandPose, demandVelocity);
}

Eigen6f LoopCycle::getTaskDemand() {
  std::lock_guard<std::mutex> lock(demandTaskMutex);
  return demandTask;
}

int64_t GetTickUs() {
  struct timespec start;
  clock_gettime(CLOCK_MONOTONIC, &start);

  return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
}

void LoopCycle::init()
{
  this->torqueMode();
  Eigen6f first_pose = Eigen6f::Zero();
  Eigen3x3f rotation = Eigen3x3f::Zero();
  Eigen3f pose = Eigen3f::Zero();
  forward_kinematics(local->getFirstPosition(),pose,rotation);
  first_pose.head(3) << -0.1,pose(1), pose(2);
  first_pose.tail(3) << rotation.eulerAngles(0,1,2);
  std::cout << "Starting pose 1 : \n " << first_pose << std::endl;
  planner->setSweepingStartingPose(first_pose);
  planner->setSweepingHeight(0.15);
  planner->setSweepingLength(0.2);
  planner->setSweepingPenetration(0);
  planner->setSweepingVelocity(0.1);
  planner->setSweepingAttackAngle(0);
  planner->init();
}


void LoopCycle::loop() {

  int timer_count = 0;
  int64_t now = 0;
  int64_t last = 0;
  try {
    RCLCPP_INFO(rclcpp::get_logger("Runner"), "Entering Torque mode...");
    init();
    RCLCPP_INFO(rclcpp::get_logger("Runner"), "Entered Torque mode.");
    // Real-time loop
    bool stop;
    node->get_parameter("lateral_run", stop);
    while (!stop) {
      now = GetTickUs();
      auto diff = now - last;
      if (diff > 1000) {
        //local->getFeedback();
        try {
          this->control();
        } 
        catch (KDetailedException &ex) 
        {
          std::cout << "Kortex exception: " << ex.what() << std::endl;
          std::cout << "Error sub-code: "
                    << SubErrorCodes_Name(SubErrorCodes(
                           (ex.getErrorInfo().getError().error_sub_code())))
                    << std::endl;
        } 
        catch (std::runtime_error &ex2) 
        {
          std::cout << "runtime error: " << ex2.what() << std::endl;
        } 
        catch (...) 
        {
          std::cout << "Unknown error." << std::endl;
        }
        timer_count++;
        last = GetTickUs();
      }
      node->get_parameter("lateral_run", stop);
    }
    RCLCPP_INFO(rclcpp::get_logger("Runner"), "breaking");

    RCLCPP_INFO(rclcpp::get_logger("Runner"), "Entering Position mode...");
    this->positionMode();
    RCLCPP_INFO(rclcpp::get_logger("Runner"), "Entered Position mode.");

  } catch (KDetailedException &ex) {
    std::cout << "API error: " << ex.what() << std::endl;
  } catch (std::runtime_error &ex2) {
    std::cout << "Error: " << ex2.what() << std::endl;
  }
}

void LoopCycle::setDemand(Eigen7f pose, Eigen7f velocity) {
  std::lock_guard<std::mutex> lock(demandMutex);
  demandPose = pose;
  demandVelocity = velocity;
}

void LoopCycle::setTaskDemand(Eigen6f pose) {
  std::lock_guard<std::mutex> lock(demandTaskMutex);
  demandTask = pose;
}

void LoopCycle::setSweepingDemand(Eigen6f pose)
{
  std::lock_guard<std::mutex> lock(demandSweepingMutex);
  demandSweeping = pose; 
}

Eigen6f LoopCycle::getSweepingDemand()
{
  std::lock_guard<std::mutex> lock(demandSweepingMutex);
  return demandSweeping;
}

void LoopCycle::setStartingPose(Eigen6f pose)
{
  std::lock_guard<std::mutex> lock(mtx_starting_pose);
  starting_pose = pose; 
}

Eigen6f LoopCycle::getStartingPose()
{
  std::lock_guard<std::mutex> lock(mtx_starting_pose);
  return starting_pose;
}