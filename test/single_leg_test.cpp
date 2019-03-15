#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "trac_ik/trac_ik.hpp"
#include "urdf/model.h"

const double PI = 3.14159;
bool flag;
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
    ros::Timer timer = n.createTimer(ros::Duration(1.0), timerCallback);
    ros::Rate loop_rate(10);
    flag = true;
    std::string robot_desc_string;
    n.param("robot_description", robot_desc_string, std::string());

    KDL::Tree my_tree;
    if(!kdl_parser::treeFromString(robot_desc_string, my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
    }

    std::vector<std::string> joint_name = {"j_coxa_1", "j_femur_1", "j_tibia_1", 
                                           "j_coxa_2", "j_femur_2", "j_tibia_2",  
                                           "j_coxa_3", "j_femur_3", "j_tibia_3", 
                                           "j_coxa_4", "j_femur_4", "j_tibia_4"};
    std::vector<double> joint_pos = {-PI/4, -PI/4.0, -PI/4.0,
                                     -PI/4, -PI/4.0, -PI/4.0,
                                     -PI/4, -PI/4.0, -PI/4.0,
                                     -PI/4, -PI/4.0, -PI/4.0};
    // for joints pos pub
    sensor_msgs::JointState joint_state;
    // for odom pub
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    // ik stuff initialization
    std::string urdf_param = "/robot_description";
    double timeout = 0.005;
    double eps = 1e-5;
    std::string chain_start = "coxa_1"; 
    std::string chain_end = "contact_sensor_1"; 
    TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);
    KDL::Chain chain;
    KDL::JntArray ll, ul; //joint lower limits, joint upper limits
    bool valid = tracik_solver.getKDLChain(chain);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL chain found");
    }
    valid = tracik_solver.getKDLLimits(ll, ul);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL joint limits found");
    }
    KDL::ChainFkSolverPos_recursive fk_solver(chain);

    ROS_INFO("joints number: %d", chain.getNrOfJoints());
    KDL::JntArray nominal(chain.getNrOfJoints()); // starting joints config
    for(size_t j = 0; j < nominal.data.size(); j ++)
    {
        // nominal(j) = (ll(j) + ul(j))/2.0;
        nominal(j) = -PI/4.0;
    }
    KDL::JntArray q(chain.getNrOfJoints()); // target joints config
    q(0) = -0.45; //rad
    q(1) = -0.45;
    q(2) = -1.57;
    KDL::Frame end_effector_pose;
    KDL::JntArray result;

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

    while(ros::ok())
    {
        // ik computation
        if(flag)
        {
            // Move the leg from nominal->end effector pose
            fk_solver.JntToCart(q, end_effector_pose); // calculate end effector pose
            // start in nominal, move to end effector pose, calculate the result for the actual ik solution
            int rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
            print_frame_lambda(end_effector_pose);
            ROS_INFO("flag");
        }
        else
        {
            // Move the leg from ee pose->nominal
            fk_solver.JntToCart(nominal, end_effector_pose); 
            int rc = tracik_solver.CartToJnt(q, end_effector_pose, result);
            print_frame_lambda(end_effector_pose);
            ROS_INFO("!flag");
        }

        // update joint_state
        ROS_INFO("update joint state");
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(12);
        joint_state.position.resize(12);
        for(int i = 0; i < 12; i ++)
        {
            joint_state.name[i] = joint_name[i];
            joint_state.position[i] = joint_pos[i];
        }
        joint_state.position[0] = result(0);
        joint_state.position[1] = result(1);
        joint_state.position[2] = result(2);
        for(int i=0;i<3;i++)
        {
            q(i)=result(i);
        }

        // update odom transform
        ROS_INFO("update odom trans");
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = x_trans;
        odom_trans.transform.translation.y = 0;
        odom_trans.transform.translation.z = 0.0866;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);

        ROS_INFO("pub joint state");
        joint_pub.publish(joint_state);
        ROS_INFO("pub odom trans");
        broadcaster.sendTransform(odom_trans);

        x_trans += 0.1;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}