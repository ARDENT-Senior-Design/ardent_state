#ifndef LEG_KINEMATIC_TREE_
#define LEG_KINEMATIC_TREE_

#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <sensor_msgs/JointState.h>
#include <boost/scoped_ptr.hpp>
#include <ardent_msgs_simple/SolveLegIk.h>

namespace ardent_model {

    #define PI 3.1415
    #define NUM_JNTS 3
    #define IS_SIM true
    class LegKinematicTree{
        // Purpose of this class is to combine the leg and body kinematics into a single robot
        private: 
            ros::NodeHandle n;
            int leg_id_;
            std::string urdf_;
            double timeout_;
            double eps_;
            
            
        public:
        // robot from the centroid to foot 
               
            KDL::Chain*                         chain_ptr_;
            KDL::ChainFkSolverPos_recursive*    fk_solver_ptr_; 
            KDL::ChainIkSolverVel_pinv*         vik_solver;
            // TRAC_IK::TRAC_IK*                   tracik_solver_ptr_;   // fastest inverse kinematic solver in the west 
            KDL::ChainIkSolverPos_NR_JL*        ik_solver_ptr_;
            KDL::JntArray                       angle_lower_bound_, 
                                                angle_upper_bound_; // joint angle lower bound and upper bound
            
            std::vector<double>                 start2end_; // start to end transformation x y z R P Y, for setEndPose preprocess 
            
            // current joint angle array
            KDL::JntArray                       jnt_pos_; 
            KDL::JntArray                       target_jnt_pos_;
            KDL::Frame                          curr_ee_pos_;
            KDL::Frame                          target_ee_pos_;

            // Ros Messaging
            ros::Publisher                      joint_pub_;
            // TODO: Change to action client
            ros::ServiceServer                  ik_service_;
            ros::ServiceServer                  fk_service_;
            std::vector<std::string>            joint_name_;

            ros::Timer                          timer; 
            /** 
             * Kinematics of a leg
             * @brief Creates a new robot
             * @param legs_ an array of the legs that will be added to the robot in order of right/left (r/l) and front (f), mid(m), and rear(r)
             */
            LegKinematicTree(int leg_id, const std::string urdf_param);
            LegKinematicTree(int leg_id, const std::string urdf_param, double timeout, double eps);
            
            ~LegKinematicTree(){ROS_INFO("Destructing Leg");}

            void initializeValues();
            void init();
            
            void setJointPose(const std::vector<double> jnt_pose);
            bool getFkSol();

            void setEndPose(const std::vector<double> pose); // x, y, z, R, P, Y
            void setBiasedEndPose(const std::vector<double> pose);
            /** 
             * Calculate the inverse kinematics 
             * @brief Creates a new robot
             * @param legs_ an array of the legs that will be added to the robot in order of right/left (r/l) and front (f), mid(m), and rear(r)
             */
            bool getIkSol(); 
            bool getIkSol(ardent_msgs_simple::SolveLegIk::Request &request, 
                            ardent_msgs_simple::SolveLegIk::Response &response);  
   
            void publishJointState();

            bool checkStability();
    }; 

}



#endif