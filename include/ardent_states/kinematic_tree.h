
#ifndef KINEMATIC_TREE_H_
#define KINEMATIC_TREE_H_


#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <string>
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <cmath>
#include <map>
#include <string>
#include <sensor_msgs/JointState.h>
#include "joint.h"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace ardent_model{

    class LegKinematics 
    {
        public:
            typedef Eigen::Vector3d Vector3d;
            
            // This class embodies a joint group object
            ros::NodeHandle n;
            robot_state::RobotStatePtr kinematic_state_;
            const robot_state::JointModelGroup* joint_model_group_;
            std::vector<std::string> joint_names_;
            std::vector<double> joint_angles_;
            std::vector<JointState*> joints_;    // list of jointsx
            std::string id_;
            Vector3d ee_pos;                    // end effector position relative to the coxa
            Vector3d ee_velocity;
            Vector3d ee_acceleration;
            Vector3d ee_force;
            /**
             * Supporting Library
             * @brief Default initialize leg lengths with values
             * @param led_id is one of the legs COXA, FEMUR, TIBIA
             * @param radial_offset is the distance from the origin to the leg position
             */
            LegKinematics(robot_model::RobotModelPtr kinematic_model, std::string id);
            ~LegKinematics();
            
            void setJointAngles();

            /**
             * @brief Retreives the current angles of the joints
             */
            void getJointAngles();
            
            /**
             * @brief Updates the target leg angle position based on an end-effector position. 
             * @param ee_pos End Effector position xyz relative to frame relative to j_c1_rf
             */ 
            void getIkCommandedAngles(Vector3d& ee_pos);
            
            /**
             * @brief Publishes positions to the coxa, femur, and tibia joint controllers
             * @param j_pos A vector that contains the target angular position of the coxa, femur, and tibia
             * */
            void publishCommandedJointState();

            /**
             * @brief Returns the cartesian position of an end effector
             * @param id Leg id for which specific leg of the six
             * @return The Cartesian position of the leg end-effector relative to the coxa
             */
            void getCartesianEndEffectorPosition();  // might be a good idea to have an ee constraint based on the angles

            /**
             * @brief Restirct the joint angles to stay within reasonable limits
             */
            void forceJointConstraints();   //TODO: Inlcude constraint based on the position of the leg. 
        
        private:
            // Angle should be kept track of by encoders, I will try to keep track of them here
            float coxa_length;
            float tibia_length;
            float femur_length;
            float radial_offset;

            ros::NodeHandle nh;
            ros::Publisher coxa_pub;
            ros::Publisher femur_pub;
            ros::Publisher tibia_pub;
            ros::Publisher contact_state_pub;
    };
}

#endif