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
    ROS_INFO("Leg Initialized");
    ROS_INFO ("Using %d joints",chain_.getNrOfJoints());

    fk_solver_ptr_->JntToCart(jnt_array_,target_frame_);

    KDL::JntArray initial_joint_pose(chain_.getNrOfJoints());

    for (uint j=0; j<initial_joint_pose.data.size(); j++) {
    initial_joint_pose(j) = 0;
  }    

    // offset the positions of the leg based on it's default position
    // {x,y,z,r,p,y}
    std::vector<std::vector<double>> com2foot {{HBDY_L, SHOU_L, -CLEG_L, 0, 0, 0},
                                                {HBDY_L, -SHOU_L, -CLEG_L, 0, 0, 0},
                                                {-HBDY_L, SHOU_L, -CLEG_L, 0, 0, 0},
                                                {-HBDY_L, -SHOU_L, -CLEG_L, 0, 0, 0}}; 
    jnt_array_(0) = 0; // shoulder joint
    jnt_array_(1) = 0;
    jnt_array_(2) = 0;
    switch(leg_id_)
    {
        case 1: start2end_ = com2foot[0]; break;
        case 2: start2end_ = com2foot[1]; break;
        case 3: start2end_ = com2foot[2]; break;
        case 4: start2end_ = com2foot[3]; break;
    }
    ROS_INFO("Leg params stored");

}

LegKinematicTree::LegKinematicTree(const int leg_id, std::string urdf):
    leg_id_(leg_id),
    urdf_(urdf),
    timeout_(0.005),
    eps_(2e-5)
{
    init();
}

LegKinematicTree::LegKinematicTree(const int leg_id, std::string _urdf_param, double _timeout, double _eps):
    leg_id_(leg_id),
    urdf_(_urdf_param),
    timeout_(_timeout),
    eps_(_eps)
{
    init();
}

void LegKinematicTree::setEndPose(const std::vector<double>& _pose) // x, y, z, R, P, Y
{
    std::vector<double> bias; 
    double x = _pose[0];
    double y = _pose[1];
    double z = _pose[2];
    double R = _pose[3];
    double P = _pose[4];
    double Y = _pose[5];
    KDL::Vector v(x, y, z);
    target_frame_ = KDL::Frame(KDL::Rotation::RPY(R, P, Y), v);
}

void LegKinematicTree::setBiasedEndPose(const std::vector<double>& _pose) // x, y, z, R, P, Y
{
    // bias shifts the origin reference point to the tip of the foot at a default position
    std::vector<double> bias; 

    double x = _pose[0]+bias[0];
    double y = _pose[1]+bias[1];
    double z = _pose[2]+bias[2];
    double R = _pose[3]+bias[3];
    double P = _pose[4]+bias[4];
    double Y = _pose[5]+bias[5];
    KDL::Vector v(x, y, z);
    target_frame_ = KDL::Frame(KDL::Rotation::RPY(R, P, Y), v);
}

bool LegKinematicTree::getIKSol(std::vector<double>& _jnts)
{
    KDL::JntArray result; 
    int rc = 0; 
    rc = tracik_solver_ptr_->CartToJnt(jnt_array_, target_frame_, result);
    ROS_INFO("\n%f",(float)rc);
    ROS_INFO("Inverse Kinematic Angles \nJoint 1: %f\nJoint 2: %f\nJoint 3: %f\n", (float)result.data[0],(float)result.data[0],(float)result.data[0]);
    if(rc < 0)
    {
        return false;
    }
    else
    {
        // clear out the joint configuration sent and replace it with the new calculated values
        _jnts.clear();
        for(size_t i = 0; i < chain_.getNrOfJoints(); i ++)
        {
            _jnts.push_back(result(i));
        }
        
        return true;
    }
}

