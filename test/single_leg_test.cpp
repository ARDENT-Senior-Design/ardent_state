#include <iostream>
#include <string>
#include <vector>

#include <tf/transform_broadcaster.h>

#include "../include/ardent_state_simple/kinematic_tree.h"
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
    ros::NodeHandle nh;   
    ros::Timer timer = nh.createTimer(ros::Duration(1), timerCallback);
    tf::TransformBroadcaster broadcaster;
    
    // Each leg must be an independent pointer so data is not lost during publishing and spinning
    std::vector<ardent_model::LegKinematicTree*> leg;
    for(int i=1;i<5;i++)
    {
        leg.push_back(new ardent_model::LegKinematicTree(i,"robot_description"));
    }
    ros::Rate loop_rate(10);
    flag = true;
    // for odom pub
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    
    ROS_INFO("joints number: %d", leg[0]->chain_.getNrOfJoints());   
   
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

    std::vector<double> jnt_pose = {PI/4,PI/4,-PI/4};
    std::vector<double> ee_pose = {0.2,-0.3,-0.4};
    while(ros::ok())
    {
        // ik computation
        if(flag)
        {
            for(int i=0;i<4;i++)
            {
                leg[i]->setEndPose(ee_pose);
                
                // Move the leg from nominal->end effector pose
                // leg[i]->fk_solver_ptr_->JntToCart(leg[i]->target_jnt_pos_, leg[i]->target_ee_pos_); // calculate end effector pose
                // start in nominal, move to end effector pose, calculate the result for the actual ik solution
                // int rc = leg[i]->tracik_solver_ptr_->CartToJnt(leg[i]->jnt_pos_, leg[i]->target_ee_pos_, leg[i]->target_jnt_pos_);
                
                leg[i]->getIkSol();
                print_frame_lambda(leg[i]->target_ee_pos_);
                ROS_INFO("flag");
                leg[i]->publishTargetJointState();
            }
        }
        else
        {
            for(int i=0;i<4;i++)
            {
                leg[i]->setJointPose(jnt_pose);
                leg[i]->getFkSol();
                // start in nominal, move to end effector pose, calculate the result for the actual ik solution
                leg[i]->getIkSol();
                print_frame_lambda(leg[i]->target_ee_pos_);
                ROS_INFO("!flag");
                leg[i]->publishTargetJointState();
            }
               
        }

        // update joint_state
        ROS_INFO("update joint state");
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