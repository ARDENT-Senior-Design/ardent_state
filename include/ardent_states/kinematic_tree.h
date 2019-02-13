
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

namespace ardent_model{
    // ardent_model is a namespace that just accumulates the  
    class RobotKinematics
    {
        public:
            std::vector<std::vector<float> > ee_jacobian; // Jacobian matrix of all links
            // handles the combination of both body and leg data? basically fuses it.
        private:
    };
    class BodyKinematics
    {
        #define BODY_THICKNESS 2
        public:
            typedef Eigen::Vector3d Vector3d;
            typedef Eigen::Matrix4d Matrix4d;
            typedef Eigen::Matrix3d Matrix3d;

            // Rotational robot state
            Matrix3d rotation; 
            Matrix3d rotation_dot;
            Matrix3d rotation_ddot;    

            // Vector position of the body relative to the odometry frame
            Vector3d body_pose;
            
            // Radius of the body
            double radius;

            /**
             * Supporting Library
             * @brief Constructor for the body frame of the robot
             */
            BodyKinematics(); //= default;
            // ~BodyKinematics(); //= default;
            
            /**
             * @brief Turns the rotation matrix into formal roll, pitch, yaw
             * 
             */
            Vector3d getRPY();

            /**
             * @brief The transform to the coxa joint of a leg based on the orientation and size of the body
             * @return The transform from the centroid of the body to the leg location based on the RPY of the robot body and relative leg coordinate
             */
            Matrix4d getLegPosition(std::string leg_id);

                /**
             * @brief Returns the 0-angle of the leg relative to the IMU direction
             * @return The angular offset of the leg regular to the X-axis
             */
            double getLegAngleOffset(std::string leg_id);

        private:
            
            
    };

    class LegKinematics 
    {
        public:
            typedef Eigen::Vector3d Vector3d;

            std::vector<JointState*> joints_;    // list of joints
            std::string leg_id;
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
            LegKinematics(std::string leg_id);
            ~LegKinematics();
            
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