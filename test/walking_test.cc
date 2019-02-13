#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "../include/ardent_states/robot.h"

float leg_num = -1;
void timerCallback(const ros::TimerEvent& event){
    if(leg_num >= 6)
    {
        leg_num = 0;
    }
    else{
        leg_num+=2;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "walking_test");
    ros::NodeHandle n;
    ardent_hardware_interface::HardwareInterface *hw;
    ardent_model::Robot ardent(hw); 
    //test
    // blah
    //Eigen::Vector3d command1 = Eigen::Vector3d(0.35,0.0,-0.25); //relative to the center of the body
    //Eigen::Vector3d command2 = Eigen::Vector3d(0.2,0.0,0);

    ros::Timer timer = n.createTimer(ros::Duration(2.0), timerCallback);
    float angle = 0;
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        /*for(int i=0;i<6;i++){
            if(i == leg_num){
                ardent.PublishLegPosition(ardent.GetMappedLeg(i),command2);
            }
            else{
                ardent.PublishLegPosition(ardent.GetMappedLeg(i), command1);
            }
        }*/
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
