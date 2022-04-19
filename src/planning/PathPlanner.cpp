#include "planning/PathPlanner.h"

void PathPlanner::setSurfaceLength(float length)
{
    this->length = length;
}

void PathPlanner::setSurfaceWidth(float width)
{
    this->width = width;
}

void PathPlanner::setSlicesCount(int slices_ct)
{
    this->slices_ct = slices_ct;
}

void PathPlanner::setStartingPoint(Eigen::Matrix<float, 3,1> _starting_point)
{
    starting_point = _starting_point; 
}

void PathPlanner::generateStartingPoints()
{

    Eigen::Matrix<float,3,1> target;
    // the robot start at the starting point
    target = starting_point;
    target_point_list.emplace_back(target);
    // the robot go down to the starting point
    for(float i = 0; i < width; i += (width / slices_ct))
    {
        target(0) += 0;
        target(1) += 0;
        target(2) += (width / slices_ct);
        target_point_list.emplace_back(target);
    }
}

void PathPlanner::resetIndex()
{
    index = 0;
}

void PathPlanner::incrementIndex()
{
    index++;
}

void PathPlanner::decrementIndex()
{
    index--;
}

Eigen::Matrix<float, 3,1> PathPlanner::getTrajectoryPoint(unsigned int index)
{
    return target_point_list.at(index);
}

unsigned int PathPlanner::getIndex()
{
    if(index > (target_point_list.size() - 1)) index = target_point_list.size() - 1;
    if(index < 0) index =  0;
    return index;
}