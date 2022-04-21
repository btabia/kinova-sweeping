/**
 * @file MicroPlanner.h
 * @author Bechir Tabia
 * @brief Micro planner algorithm for sweeping task. 
 * @version 0.1
 * @date 2022-03-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include "eigen3/Eigen/Dense"
#include "Utils.h"
#include <vector>
#include <iostream>


// class allowing to generate a oval shape trajectory for the brush
class MicroPlanner
{
    public: 

    MicroPlanner():
    length(0),
    height(0),
    penetration(0),
    velocity(0),
    loop_cycle_freq(1000),
    attack_angle(0),
    orientation(0,0,0),
    index(0)
    {
        starting_pose << 0,0,0,0,0,0;
    }
    ~MicroPlanner(){}

    /************** setters **************/
    
    /**
     * @brief Set the Length object
     * 
     * @param length 
     */
    void setLength(float length);
    /**
     * @brief Set the Height object
     * 
     * @param height 
     */
    void setHeight(float height);
    /**
     * @brief Set the Penetration object
     * 
     * @param penetration 
     */
    void setPenetration(float penetration); 
    /**
     * @brief Set the Velocity object
     * 
     * @param velocity 
     */
    void setVelocity(float velocity);
    /**
     * @brief Set the Loop Cycle Freq object
     * 
     * @param loop_cycle_freq 
     */
    void setLoopCycleFreq(float loop_cycle_freq);
    /**
     * @brief Set the Attack Angle object
     * 
     * @param attack_angle in degrees
     */
    void setAttackAngle(float attack_angle);
    /**
     * @brief Set the Starting Pose object
     * 
     * @param starting_pose 
     */
    void setStartingPose(Eigen6f starting_pose);

    /************** getters **************/
     /**
     * @brief Get the Trajectory Point object
     * 
     * @param index 
     * @return Eigen::Vector3d 
     */
    Eigen6f getTrajectoryPoint(unsigned int index);
    /**
     * @brief Get the Trajectory Size object
     * 
     * @return unsigned int 
     */
    unsigned int getTrajectorySize();
    /**
     * @brief Get the Index object
     * 
     * @return unsigned int 
     */
    unsigned int getIndex();
    /**
     * @brief Get the Attack Angle object
     * 
     * @return float 
     */
    float getAttackAngle();

    /**
     * @brief Get the Orientation object
     * 
     * @return Eigen::Quaternionf 
     */
    Eigen::Vector3f getOrientation();

    /************** functions **************/
    /**
     * @brief Generate the list of point to generate the sweeping motion
     * 
     * @param _centralPose 
     */
    void generateTrajectory();
    /**
     * @brief increment the index
     * 
     */
    void incrementIndex();
    /**
     * @brief decrement the index
     * 
     */
    void decrementIndex();
    /**
     * @brief reset the index
     * 
     */
    void resetIndex();
    /**
     * @brief print the index value
     * 
     */
    void printIndex();


    private:

    float length;
    float height; 
    float penetration; 
    float velocity;
    float loop_cycle_freq;
    float attack_angle;

    /**
     * @brief list of generated trajectory
     * 
     */
    std::vector<Eigen6f> trajectory_list;
    Eigen::Vector3f orientation;

    Eigen6f starting_pose;

    unsigned int index; // sequence index

    unsigned int turn_count;

};
