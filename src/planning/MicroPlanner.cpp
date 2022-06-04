#include "planning/MicroPlanner.h"

/************** setters **************/

void MicroPlanner::setLength(float length)
{
    this->length = length; 
}

void MicroPlanner::setHeight(float height)
{
    this->height = height;
}

void MicroPlanner::setPenetration(float penetration)
{
    this->penetration = penetration; 
}

void MicroPlanner::setVelocity(float velocity)
{
    this->velocity = velocity; 
}

void MicroPlanner::setLoopCycleFreq(float loop_cycle_freq)
{
    this->loop_cycle_freq = loop_cycle_freq; 
}

void MicroPlanner::setAttackAngle(float attack_angle)
{
    this->attack_angle = attack_angle; 
    orientation << 0,0,attack_angle;
}

void MicroPlanner::setStartingPose(Eigen6f starting_pose)
{
    this->starting_pose = starting_pose;
}


/************** getter **************/
Eigen6f MicroPlanner::getTrajectoryPoint(unsigned int index)
{
    return trajectory_list.at(index);
}

unsigned int MicroPlanner::getTrajectorySize()
{
    return trajectory_list.size();
}

unsigned int MicroPlanner::getIndex()
{
    return index;
}

float MicroPlanner::getAttackAngle()
{
    return attack_angle;
}

Eigen::Vector3f MicroPlanner::getOrientation()
{
    return orientation;
}

/************** function **************/

/**** orientation for 90 degrees angle of attack******/
/*

  3.08869
  1.57064
0.0528876

x_theta = 3.08
y_theta = 1.57
z_theta = 0.05288976

*/

/**** orientation for 80 degrees angle of attack******/
/*

  1.56961
  1.75762
  1.57204

   x_theta = 1.57
    y_theta = 1,74533
    z_theta = 1.57

*/

/**** orientation for 70 degrees angle of attack******/
/*

   1.57017
   1.96734
   1.57147

   x_theta = 1.57017
    y_theta = 1.96734
    z_theta = 1.57147

*/

/**** orientation for 60 degrees angle of attack******/
/*

   1.57015
   2.09665
   1.57163

*/


void MicroPlanner::generateTrajectory()
{
        Eigen6f command = Eigen6f::Zero();
        float increment = velocity / loop_cycle_freq ;
        // set up starting point
        command << starting_pose;
        command(0) = command(0);
        command(1) = command(1);
        command(2) = command(2);
        command(3) = 1.57;
        command(4) = 1.57 + (1.57 - attack_angle);
        command(5) = 1.57;
        trajectory_list.emplace_back(command);

        for(float i = 0; i < length; i += increment)
        {
            float delta_x = 0;
            float delta_y = increment;
            float delta_z = 0;

            command(0) += delta_x; // x axis
            command(1) += delta_y;
            command(2) += delta_z; // z axis
            //command.tail(3) << starting_pose.tail(3);
            trajectory_list.emplace_back(command);
                              
        }

        for(float i = 0; i < (height + penetration); i+= increment)
        {
            float delta_x = increment; // height
            float delta_y = 0;
            float delta_z = 0; 

            command(0) += delta_x; // x axis
            command(1) += delta_y;
            command(2) += delta_z; // z axis
            //command.tail(3) << starting_pose.tail(3);
            trajectory_list.emplace_back(command);

        }

        for(float i = 0; i < length; i+= increment)
        {
            float delta_x = 0;
            float delta_y = -increment;
            float delta_z = 0;

            command(0) += delta_x; // x axis
            command(1) += delta_y;
            command(2) += delta_z; // z axis
            //command.tail(3) << starting_pose.tail(3);
            trajectory_list.emplace_back(command);

        }

        for(float i = 0; i < height; i+= increment)
        {
            float delta_x = -increment;
            float delta_y = 0;
            float delta_z = 0; 

            command(0) += delta_x; // x axis
            command(1) += delta_y;
            command(2) += delta_z; // z axis
            //command.tail(3) << starting_pose.tail(3);
            trajectory_list.emplace_back(command);

        }



}



void MicroPlanner::incrementIndex()
{
    index++;
}

void MicroPlanner::decrementIndex()
{
    index--;
}


void MicroPlanner::resetIndex()
{
    index = 0;
}


void MicroPlanner::printIndex()
{
    std::cout << "Index value: " << index << std::endl;
}