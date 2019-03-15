#include <iostream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "urdf/model.h"

#include "../include/ardent_state_simple/kinematic_tree.h"

using namespace ardent_model;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "walking_test");
    ros::NodeHandle n;

    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(10);

    std::string robot_desc_string;
    n.param("robot_description", robot_desc_string, std::string());

    std::vector<std::string> joint_name = {"j_coxa_1", "j_femur_1", "j_tibia_1", 
                                           "j_coxa_2", "j_femur_2", "j_tibia_2",  
                                           "j_coxa_3", "j_femur_3", "j_tibia_3", 
                                           "j_coxa_4", "j_femur_4", "j_tibia_4"};
    
    // // for joints pos pub
    sensor_msgs::JointState joint_state;
    // // for odom pub
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    // // initialize four legs
    LegKinematicTree leg_lf(1);
    // LegIK leg_rf("rf");
    // LegIK leg_lb("lb");
    // LegIK leg_rb("rb");
    std::vector<LegKinematicTree> legs{leg_lf};//, leg_rf, leg_lb, leg_rb};
    ROS_INFO("Leg's calibrated");
    std::vector<std::vector<double>> end_pose(3, std::vector<double>(6, 0.0)); // 4 chain, end pose has 6dof
    std::vector<std::vector<double>> result(1, std::vector<double>(3, 0.0)); // 4 legs, every leg has 3 joints 
   
    bool first = true;
    int flag = -1;
    int cycle_cnt = 0; // walk cycle cnt
    double CYCLE_L = 0.015; // (1/4)cyle length
    double STEP_L = 0.001 ; // move distance in every loop
    double cycle_trans = 0;
    double x_trans_total = 0;
    // for base_link pose
    double x_trans = 0;
    double y_trans = 0;
    double z_trans = 0;
    // for shoe_link pose
    double x_trans_shoe = 0;
    double y_trans_shoe = 0;
    double z_trans_shoe = 0;

    double error = 0.00001;
    while(ros::ok())
    {
        x_trans += STEP_L;
        cycle_trans += STEP_L;
        x_trans_total = cycle_trans + cycle_cnt * CYCLE_L;
    //     if(!first)
    //     {
    //         x_trans_shoe += STEP_L;
    //     }
    //     if(cycle_trans > CYCLE_L)
    //     {
    //         cycle_cnt ++;
    //         cycle_trans = 0;
    //     }
    //     if(x_trans > 2*CYCLE_L )
    //     {
    //         x_trans = 0.0;
    //         x_trans_shoe = -2*CYCLE_L;
    //         z_trans_shoe = 0.0;
    //         first = false;
    //     }

    //     ROS_INFO("cycle #%d", cycle_cnt);

    //     // set leg status: up2down or down2up
    //     if(cycle_cnt%4 == 0 || cycle_cnt%4 == 1)
    //     {
    //         legs[0].setUp2Down(true); 
    //         legs[1].setUp2Down(false); 
    //         legs[2].setUp2Down(false); 
    //         legs[3].setUp2Down(true); 
    //     }
    //     else
    //     {
    //         legs[0].setUp2Down(false); 
    //         legs[1].setUp2Down(true); 
    //         legs[2].setUp2Down(true); 
    //         legs[3].setUp2Down(false); 
    //     }

    //     // set end pose
    //     if(cycle_cnt%4 == 0 || cycle_cnt%4 == 2)
    //     {
    //         z_trans_shoe += STEP_L;
    //     }
    //     else
    //     {
    //         z_trans_shoe -= STEP_L;
    //     }
    //     ROS_INFO("x_trans_total: %lf", x_trans_total);
        ROS_INFO("x_trans: %lf", x_trans);
    //     ROS_INFO("x_trans_shoe: %lf", x_trans_shoe);
    //     ROS_INFO("z_trans_shoe: %lf", z_trans_shoe);

        for(int i = 0; i < 1; i++)
        {
            std::vector<double> pose{x_trans, y_trans, z_trans, 0, 0, 0}; // x y z r y p 
            end_pose[i] = pose;
        }

        // calculate the IK
        for(int i = 0; i < 1; i++)
        {
            legs[i].setEndPose(end_pose[i]);
            ROS_INFO("End Pose %i: \nx: %f \ny: %f \nz: %f",i,end_pose[i][0],end_pose[i][1],end_pose[i][2]);
            if(!legs[i].getIKSol(result[i]))
            {
                std::cout <<"not success" << std::endl;
                // return -1;
            }
            else
            {
                std::cout << "success" << std::endl;
            }
        }

        // update joint_state
        // ROS_INFO("update joint state");
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(JNTS_NUM);
        joint_state.position.resize(JNTS_NUM);
        std::vector<double> new_pos;
        for(int i = 0; i < 1; i ++)
        {
            for(int j = 0; j < 3; j++)
            {
                // add the new updated position to the list to update the joints
                // new_pos.push_back(result[i][j]);
                new_pos.push_back(0);
            } 

        } 
        for(size_t i = 0; i < JNTS_NUM; i ++) 
        { 
            joint_state.name[i] = joint_name[i]; 
            joint_state.position[i] = new_pos[i]; 
        } 
        
        // update odom transform ROS_INFO("update odom trans");
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = x_trans_total;
        odom_trans.transform.translation.y = y_trans;
        odom_trans.transform.translation.z = CLEG_L + z_trans;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);

    //     ROS_INFO("pub joint state");
        joint_pub.publish(joint_state);
    //     ROS_INFO("pub odom trans");
        broadcaster.sendTransform(odom_trans);

        loop_rate.sleep();
       
    }
}
