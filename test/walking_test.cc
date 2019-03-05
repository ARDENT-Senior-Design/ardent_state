#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "../include/ardent_states/robot.h"

double angle;
void timerCallback(const ros::TimerEvent& event){
    ROS_INFO("MOVING THE LEG");
    if(angle == 0)
    {
        angle = 1;
    }
    else
    {
        angle *=-1;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "walking_test");
    ardent_model::Robot ardent("robot_description");
    ros::NodeHandle n;
    angle = 0;
    //test
    // blah
    //Eigen::Vector3d command1 = Eigen::Vector3d(0.35,0.0,-0.25); //relative to the center of the body
    //Eigen::Vector3d command2 = Eigen::Vector3d(0.2,0.0,0);

    ros::Timer timer = n.createTimer(ros::Duration(2.0), timerCallback);
    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
        ardent.legs_[0]->getJointAngles();
        ardent.legs_[0]->joint_angles_[1] = angle;
        ardent.legs_[0]->setJointAngles();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
