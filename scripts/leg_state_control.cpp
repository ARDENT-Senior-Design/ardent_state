#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "trac_ik/trac_ik.hpp"
#include "urdf/model.h"
#include "../include/ardent_state_simple/leg_kinematic_tree.h"
// #include "../include/ardent_state_simple/robot.h"

bool flag;
bool prev_flag;
void timerCallback(const ros::TimerEvent&)
{
    flag = !flag;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ik_test_node");
    // ros::NodeHandle* n = new ros::NodeHandle();
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
    tf::TransformBroadcaster broadcaster;

    ROS_INFO("Ros Messages Linked");   

    ardent_model::LegKinematicTree* leg[4];
    leg[0] = new ardent_model::LegKinematicTree(1,"robot_description");
    leg[1] = new ardent_model::LegKinematicTree(2,"robot_description");
    leg[2] = new ardent_model::LegKinematicTree(3,"robot_description");
    leg[3] = new ardent_model::LegKinematicTree(4,"robot_description");
    
    ROS_INFO("Legs Initialized");   

    ros::Timer timer = n.createTimer(ros::Duration(1.0), timerCallback);
    ros::Rate loop_rate(5);

    flag = true;

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    ROS_INFO("joints number: %d", leg[0]->chain_ptr_->getNrOfJoints());   
   
    double x_trans = 0;
    
    auto print_frame_lambda = [](KDL::Frame f)
    {
        double x, y, z, roll, pitch, yaw;
        x = f.p.x();
        y = f.p.y();
        z = f.p.z();
        f.M.GetRPY(roll, pitch, yaw);
        std::cout << "x:" << x << " y:" << y << " z:" << z << " roll:" << roll << " pitch:" << pitch << " yaw:" << yaw << std::endl;
    };


    // std::vector<double> jnt_pose = {-PI/4,-PI/4,-PI/4};
    std::vector<double> ee_pose = {0.3,-0.25,-0.0};
    ROS_INFO("Beginning Leg State Update");
    int polarity = 1;
   

    // BACK: x:0.189534 y:-0.314737 z:-0.425656 roll:0.0581101 pitch:1.57 yaw:-3.00525
    // MID "front": x:0.319438 y:-0.248891 z:-0.425656 roll:0.0581101 pitch:1.57 yaw:-2.21987
    // FRONT: 
    while(ros::ok())
    {

        auto start = std::chrono::high_resolution_clock::now();
         // ik computation
        // if(flag)
        // {
       /* for(int i=0;i<4;i++)
        {                                                                                                        
            if(i>=2)
            {
                ee_pose[1]=-abs(ee_pose[1]);
                // ee_pose[1]=-abs(ee_pose[1]);
            }
            leg[i]->setEndPose(ee_pose);
            leg[i]->getIkSol();
            // Move the leg from nominal->end effector pose
            // leg[i]->fk_solver_ptr_->JntToCart(leg[i]->target_jnt_pos_, leg[i]->target_ee_pos_); // calculate end effector pose
            // start in nominal, move to end effector pose, calculate the result for the actual ik solution
            // int rc = leg[i]->tracik_solver_ptr_->CartToJnt(leg[i]->jnt_pos_, leg[i]->target_ee_pos_, leg[i]->target_jnt_pos_);
            
           
            print_frame_lambda(leg[i]->target_ee_pos_);
            
        }
        if(flag)
        {
            ROS_INFO("flag");
            ee_pose = {0.3,0,-0.4};
            
        }
        else
        {
            ee_pose = {0.3,0.3,-0.4};
        }*/

        // update joint_state
        
        ROS_INFO("update joint state");
        // for(int i=0;i<4;i++){
        //     leg[i]->publishJointState();
        // }
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = (finish-start);
        std::cout << "Time Elapsed calculating one leg: " << elapsed.count() << " seconds.";
        std::cout << std::endl;
        // update odom transform
        ROS_INFO("update odom trans");
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = x_trans;
        odom_trans.transform.translation.y = 0;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);

        ROS_INFO("pub odom trans");
        broadcaster.sendTransform(odom_trans);

        x_trans+= 0.001;
        prev_flag = flag;
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}