#include "../include/ardent_state_simple/kinematic_tree.h"

using namespace ardent_model;

void LegKinematicTree::init()
{
    chain_start_ = "base_link";
    chain_end_ = "contact_sensor_"+std::to_string(leg_id_);
    tracik_solver_ptr_ = new TRAC_IK::TRAC_IK(chain_start_, chain_end_,urdf_, timeout_,eps_);
    bool valid = tracik_solver_ptr_->getKDLChain(chain_);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL solution found");
    }
    valid = tracik_solver_ptr_->getKDLLimits(angle_lower_bound_, angle_upper_bound_);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL joint limits found");
    }
    fk_solver_ptr_ = new KDL::ChainFkSolverPos_recursive(chain_);
    jnt_array_ = KDL::JntArray(chain_.getNrOfJoints());
    
    // offset the positions of the leg based on it's position
    std::vector<std::vector<double>> com2foot {{HBDY_L, SHOU_L, -CLEG_L, 0, 0, 0},
                                                {HBDY_L, -SHOU_L, -CLEG_L, 0, 0, 0},
                                                {-HBDY_L, SHOU_L, -CLEG_L, 0, 0, 0},
                                                {-HBDY_L, -SHOU_L, -CLEG_L, 0, 0, 0}}; 
    jnt_array_(0) = 0; // shoulder joint
    jnt_array_(1) = PI/6.0;
    jnt_array_(2) = -PI/3.0;
    jnt_array_(3) = -PI/3.0;
    jnt_array_(4) = 0; // shoe joint
    switch(leg_id_)
    {
        case 1: start2end_ = com2foot[0]; break;
        case 2: start2end_ = com2foot[1]; break;
        case 3: start2end_ = com2foot[2]; break;
        case 4: start2end_ = com2foot[3]; break;
    }
}