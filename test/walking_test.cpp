#include "ros/ros.h"
#include "std_msgs/Float64.h"
// #include "../include/ardent_states/robot.h"

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
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
