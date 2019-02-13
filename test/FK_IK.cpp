#include "../include/ardent_states/kinematic_tree.h"
#include <urdf/model.h>
#include <ros/ros.h>
#include <iostream>
int main(int argc, char **argv)
{
    // This test will create the forward and inverse kinematic information for the robot to verify it
    ros::init(argc, argv, "FK_IK");

    // if (argc != 2){
    //     ROS_ERROR("Need a urdf file as argument");
    //     return -1;
    // }
    // std::string urdf_file = argv[1];
    std::string urdf_file = "~/ardent_ws/src/ardent_description/config/robot/urdf/ardent.urdf";
    urdf::Model model;
    if (!model.initFile(urdf_file)){
        ROS_ERROR("Failed to parse urdf file");
        return -1;
    }
    ROS_INFO("Successfully parsed urdf file");
    
    ros::NodeHandle n;
    return 0;
    //test
    // blah
    //Eigen::Vector3d command1 = Eigen::Vector3d(0.35,0.0,-0.25); //relative to the center of the body
    //Eigen::Vector3d command2 = Eigen::Vector3d(0.2,0.0,0);

    // ros::Timer timer = n.createTimer(ros::Duration(2.0), timerCallback);
    // float angle = 0;
    // ros::Rate loop_rate(10);

    // while(ros::ok())
    // {
        
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

}