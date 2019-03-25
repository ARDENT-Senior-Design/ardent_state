#include "../include/ardent_state_simple/robot.h"
using namespace ardent_model;

Robot::Robot()
{
    if(init("robot_description")
    {
        ROS_INFO("Robot Initialized");
    }
}

bool init(std::string urdf)
{
    if(!n.param(urdf, urdf_))
    {
        ROS_FATAL("Could not load XML from param: robot_description");
        return false;
    }

}
// bool calculateKinematics (crab_msgs::BodyState* body_ptr);
// bool callService (KDL::Vector* vector);
// void teleopBodyMove (const crab_msgs::BodyStateConstPtr &body_state);
// void teleopBodyCmd (const crab_msgs::BodyCommandConstPtr &body_cmd);