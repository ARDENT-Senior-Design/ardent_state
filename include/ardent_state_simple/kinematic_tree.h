#ifndef ROBOT_H_
#define ROBOT_H_

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <urdf/model.h>
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "trac_ik/trac_ik.hpp"

#include <boost/scoped_ptr.hpp>

namespace ardent_model {

    #define PI 3.14159
    #define SHOU_L 0.04
    #define CLEG_L 0.086
    #define HBDY_L 0.05 
    #define JNTS_NUM 12

    class LegKinematicTree{
        // Purpose of this class is to combine the leg and body kinematics into a single robot
        private: 
            int leg_id_;
            std::string urdf_;
            double timeout_;
            double eps_;
            // robot from the centroid to foot
            KDL::Chain chain_;
            TRAC_IK::TRAC_IK* tracik_solver_ptr_;   // fastest inverse kinematic solver in the west
            KDL::ChainFkSolverPos_recursive* fk_solver_ptr_;
            KDL::JntArray jnt_array_; // current joint angle array
            KDL::JntArray angle_lower_bound_, angle_upper_bound_; // joint angle lower bound and upper bound
            std::string chain_start_;
            std::string chain_end_;
            std::vector<double> start2end_; // start to end transformation x y z R P Y, for setEndPose preprocess 
            
            KDL::Frame target_frame_;
        public:
            ros::NodeHandle n;
            
            ros::Subscriber robot_cmd;
            ros::Subscriber robot_motion;

            /** 
             * Kinematics of a leg
             * @brief Creates a new robot
             * @param legs_ an array of the legs that will be added to the robot in order of right/left (r/l) and front (f), mid(m), and rear(r)
             */
            LegKinematicTree(const int leg_id);
            LegKinematicTree(const int leg_id, std::string urdf_param, double timeout, double eps);
            ~LegKinematicTree(){};
            void init();
            void setEndPose(const std::vector<double>& _pose); // x, y, z, R, P, Y
            
            /** 
             * Calculate the inverse kinematics 
             * @brief Creates a new robot
             * @param legs_ an array of the legs that will be added to the robot in order of right/left (r/l) and front (f), mid(m), and rear(r)
             */
            bool getIKSol(std::vector<double>& _jnts);

            bool checkStability();
    }; 

}



#endif