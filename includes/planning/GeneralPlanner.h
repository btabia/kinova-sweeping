#pragma once

#include "MicroPlanner.h"
#include "GeneralPlanner.h"
#include "Utils.h"
#include <iostream>

/// \cond
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "KinovaControl.h"
/// \endcond 

class GeneralPlanner
{
    public: 
        GeneralPlanner(std::string name_, 
                rclcpp::Node::SharedPtr node_): 
        name(name_),
        node(node_), 
        index(0)
        {
            position_demand << 0,0,0,0,0,0;
            std::map<std::string, float> params; 

            params["sweeping_length"] = sweeping_length; 
            params["sweeping_height"] = sweeping_height; 
            params["sweeping_penetration"] = sweeping_penetration; 
            params["sweeping_velocity"] = sweeping_velocity;
            params["sweeping_attack_angle"] = sweeping_attack_angle;
            for(unsigned int i = 0; i < 3; ++i)
            {
                params["/axis_" + std::to_string(i) + "/starting_pose"] = sweeping_starting_pose[i];
            }

            node->declare_parameters(name, params);
        }
        ~GeneralPlanner() {}

        void init();

        void generateSweepingTrajectory();

        Eigen6f getPositionOrientationDemand();

        void respond();

        //********* Setter *********//
        void setSweepingLength(float input)
        {
            std::lock_guard<std::mutex> lk(mtx_sweeping_length);
            sweeping_length = input;
        }

        void setSweepingHeight(float input)
        {
            std::lock_guard<std::mutex> lk(mtx_sweeping_height);
            sweeping_height = input;
        }

        void setSweepingPenetration(float input)
        {
            std::lock_guard<std::mutex> lk(mtx_sweeping_penetration);
            sweeping_penetration = input;
        }

        void setSweepingStartingPose(Eigen6f input)
        {
            std::lock_guard<std::mutex> lk(mtx_sweeping_starting_pose);
            sweeping_starting_pose = input;
        }

        void setSweepingVelocity(float input)
        {
            std::lock_guard<std::mutex> lk(mtx_sweeping_velocity);
            sweeping_velocity = input;
        }

        void setSweepingAttackAngle(float input)
        {
            std::lock_guard<std::mutex> lk(mtx_sweeping_attack_angle);
            sweeping_attack_angle = input DEG2RAD;
        }

        void setTurnCount(unsigned int count)
        {
            turn_count = count;
        }

        //********* Getter *********//

        float getSweepingLength()
        {
            std::lock_guard<std::mutex> lk(mtx_sweeping_length);
            return sweeping_length; 
        }

        float getSweepingHeight()
        {
            std::lock_guard<std::mutex> lk(mtx_sweeping_height);
            return sweeping_height; 
        }

        float getSweepingPenetration()
        {
            std::lock_guard<std::mutex> lk(mtx_sweeping_penetration);
            return sweeping_penetration;
        }

        Eigen6f getSweepingStartingPose()
        {
            std::lock_guard<std::mutex> lk(mtx_sweeping_starting_pose);
            return sweeping_starting_pose;
        }

        float getSweepingVelocity()
        {
            std::lock_guard<std::mutex> lk(mtx_sweeping_velocity);
            return sweeping_velocity;
        }

        float getSweepingAttackAngle()
        {
            std::lock_guard<std::mutex> lk(mtx_sweeping_attack_angle);
            return sweeping_attack_angle;
        }

        unsigned int getTurnCount()
        {
            return turn_count;
        }

    private:
        /**
         * @brief Get the Micro Position object
         * 
         * @return Eigen6f 
         */
        Eigen6f getMicroPosition();

    private: 

        Eigen6f position_demand;
 
        MicroPlanner micro_planner; 

        float sweeping_length; 
        float sweeping_height; 
        float sweeping_penetration; 
        Eigen6f sweeping_starting_pose;
        float sweeping_velocity;
        float sweeping_attack_angle;
        unsigned int index;
        unsigned int turn_count;

        std::mutex mtx_sweeping_length;
        std::mutex mtx_sweeping_height;
        std::mutex mtx_sweeping_penetration;
        std::mutex mtx_sweeping_starting_pose; 
        std::mutex mtx_sweeping_velocity;
        std::mutex mtx_sweeping_attack_angle;

        std::string name;

        rclcpp::Node::SharedPtr node;
        
};