#pragma once

#include <iostream>
#include <vector>
#include "Utils.h"

class PathPlanner
{
    public:
        PathPlanner():
        length(0), 
        width(0), 
        height(0), 
        slices_ct(0),
        index(0),
        starting_point(0,0,0)
        {

        }
        ~PathPlanner() {}

        /**
         * @brief Set the Surface Length object
         * 
         * @param length 
         */
        void setSurfaceLength(float length);
        /**
         * @brief Set the Surface Width object
         * 
         * @param width 
         */
        void setSurfaceWidth(float width);
        /**
         * @brief Set the Slices Count object
         * 
         * @param slices_ct 
         */
        void setSlicesCount(int slices_ct);
        /**
         * @brief Set the Starting Point object
         * 
         * @param starting_point 
         */
        void setStartingPoint(Eigen::Matrix<float, 3,1> _starting_point);

        /**
         * @brief function generating the starting point for each test
         * 
         */
        void generateStartingPoints();

        /**
         * @brief reset the index to zero
         * 
         */
        void resetIndex();

        /**
         * @brief increment the index value;
         * 
         */
        void incrementIndex();

        /**
         * @brief decrement the index value
         * 
         */
        void decrementIndex();

        /**
         * @brief Get the Path Point object
         * 
         * @return Eigen::Matrix<float, 3,1> 
         */
        Eigen::Matrix<float, 3,1> getTrajectoryPoint(unsigned int index);
        /**
         * @brief Get the Index object
         * 
         * @return unsigned int 
         */
        unsigned int getIndex();


    private:

        // parameter
        float length, width, height; 
        int slices_ct; 
        unsigned int index;

        // starting point
        Eigen::Matrix<float, 3,1> starting_point; // starting position and orientation

        std::vector<Eigen::Matrix<float,3,1> > target_point_list;
};