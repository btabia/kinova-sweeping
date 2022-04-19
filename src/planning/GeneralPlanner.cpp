#include "planning/GeneralPlanner.h"
#include <rclcpp/rclcpp.hpp>


void GeneralPlanner::init()
{
    // init the micro planner component
    micro_planner.setLength(sweeping_length); // 10 cm travel
    micro_planner.setHeight(sweeping_height); // 5cm travel
    micro_planner.setPenetration(sweeping_penetration); // 2cm travel
    micro_planner.setVelocity(sweeping_velocity); // 0.1 m/s
    micro_planner.setLoopCycleFreq(1000); // 1Khz
    micro_planner.setAttackAngle(sweeping_attack_angle); // -45 degrees on axis Z
    micro_planner.setStartingPose(sweeping_starting_pose);
    micro_planner.generateTrajectory();

    RCLCPP_INFO(rclcpp::get_logger("Planning"), "Planning initialisation complete");
}


Eigen6f GeneralPlanner::getPositionOrientationDemand()
{
    Eigen6f demand = getMicroPosition();
    //std::cout << "demand coordinate: \n" << demand <<std::endl;
    return demand;
}

void GeneralPlanner::respond()
{
    std::map<std::string, float> params;
    node->get_parameters(name, params);

    float sweeping_length_ = params["sweeping_length"]; 
    float sweeping_height_ = params["sweeping_height"]; 
    float sweeping_penetration_ = params["sweeping_penetration"]; 
    Eigen6f sweeping_starting_pose_;
    float sweeping_velocity_ = params["sweeping_velocity"];
    float sweeping_attack_angle_ = params["sweeping_attack_angle"];

    for(unsigned int i = 0; i < 6; ++i)
    {
        sweeping_starting_pose[i] = params["/axis_" + std::to_string(i) + "/starting_pose"];
    }

    setSweepingHeight(sweeping_height_);
    setSweepingLength(sweeping_length_);
    setSweepingPenetration(sweeping_penetration_);
    setSweepingStartingPose(sweeping_starting_pose_);
    setSweepingVelocity(sweeping_velocity_);
    setSweepingAttackAngle(sweeping_attack_angle_);

    
}

Eigen6f GeneralPlanner::getMicroPosition()
{
    Eigen6f demand;
    demand = micro_planner.getTrajectoryPoint(micro_planner.getIndex());
    // update commands
    /*if((micro_planner.getIndex() < micro_planner.getTrajectorySize()) && (micro_planner.getTurn() == false))
    {
        micro_planner.incrementIndex();
        if(micro_planner.getIndex() == (micro_planner.getTrajectorySize() - 1))
        {
            micro_planner.resetIndex();
            //micro_planner.setTurn(true);
        }
    }

    if((micro_planner.getIndex() > 0) && (micro_planner.getTurn() == true))
    {
        micro_planner.decrementIndex();
        if(micro_planner.getIndex() == 0)
        {
            //micro_planner.resetIndex();
            //micro_planner.setTurn(false);
        }
    }*/
    if(micro_planner.getIndex() < micro_planner.getTrajectorySize())
    {
        micro_planner.incrementIndex();
        if(micro_planner.getIndex() == (micro_planner.getTrajectorySize() - 1))
        {
            micro_planner.resetIndex();
            //micro_planner.setTurn(true);
        }
    }

    /*if(micro_planner.getIndex() > 0)
    {
        micro_planner.decrementIndex();
        if(micro_planner.getIndex() == 0)
        {
            //micro_planner.resetIndex();
            //micro_planner.setTurn(false);
        }
    }*/
    return demand;
}

