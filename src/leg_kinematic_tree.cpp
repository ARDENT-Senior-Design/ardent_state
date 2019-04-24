#include "../include/ardent_state_simple/leg_kinematic_tree.h"


using namespace ardent_model;

    

    void LegKinematicTree::initializeValues()
    {
        angle_lower_bound_.resize(NUM_JNTS);
        angle_upper_bound_.resize(NUM_JNTS);
        jnt_pos_.resize(NUM_JNTS);
        target_jnt_pos_.resize(NUM_JNTS);
        for(int i=0;i<NUM_JNTS;i++)
        {
            angle_lower_bound_.data[i]=-(PI/2);
            angle_upper_bound_.data[i]=(PI/2);
            jnt_pos_.data[i]= 0;
            target_jnt_pos_.data[i]= 0;
        }
        joint_pub_ = n.advertise<sensor_msgs::JointState>("joint_states", 10);
    }

    void LegKinematicTree::init()
    {
        std::string robot_desc_string;
        n.param(urdf_, robot_desc_string, std::string());
        KDL::Tree tree;
        ROS_INFO("Constructing Tree");    
        if(!kdl_parser::treeFromString(robot_desc_string, tree))
        {
            ROS_ERROR("Failed to construct kdl tree");
        }
        ROS_INFO("Constructed tree");

        std::string leg_id = std::to_string(leg_id_);
        joint_name_ = {"j_coxa_"+leg_id, "j_femur_"+leg_id, "j_tibia_"+leg_id};
        const std::string chain_start_ = "base_link";
        const std::string chain_end_ = "contact_sensor_"+leg_id;

        KDL::Chain chain;
        if (!tree.getChain(chain_start_, chain_end_, chain)) {
            ROS_ERROR("Could not initialize chain");
        }
        chain_ptr_ = new KDL::Chain(chain);
        ROS_INFO("Constructed chains");

        initializeValues();
        // Build Solvers
        fk_solver_ptr_ = new KDL::ChainFkSolverPos_recursive(*chain_ptr_);
        vik_solver = new KDL::ChainIkSolverVel_pinv(*chain_ptr_); // PseudoInverse vel solver
        ik_solver_ptr_ = new KDL::ChainIkSolverPos_NR_JL(*chain_ptr_,angle_lower_bound_,angle_upper_bound_,*fk_solver_ptr_, *vik_solver, 100, eps_); // Joint Limit Solver

        // tracik_solver_ptr_ = new TRAC_IK::TRAC_IK(chain_start_, chain_end_, urdf_, timeout_, eps_);
        // TRAC_IK::TRAC_IK* tracik_solver_ptr_ = new TRAC_IK::TRAC_IK(chain_start_, chain_end_, urdf_, timeout_, eps_);
        // bool valid = tracik_solver_ptr_->getKDLChain(chain_);
        // if(!valid)
        // {
        //     ROS_ERROR("There was no valid KDL solution found");
        // }
        // valid = tracik_solver_ptr_->getKDLLimits(angle_lower_bound_, angle_upper_bound_);
        // if(!valid)
        // {
        //     ROS_ERROR("There was no valid KDL joint limits found");
        // }

        jnt_pos_ = KDL::JntArray(chain_ptr_->getNrOfJoints());
        ROS_INFO("Joints Initialized:");    
        for(int i =0;i<3;i++)
        {
            ROS_INFO("%s \n", joint_name_[i].c_str());
        }
        ROS_INFO("Leg Initialized");
        ROS_INFO ("Using %d joints",chain_ptr_->getNrOfJoints());
        
        std::vector<double> jnt_pose = {0,0,0};
        setJointPose(jnt_pose);
        getFkSol();
        publishJointState();
        ik_service_ = n.advertiseService("get_ik_"+leg_id, &LegKinematicTree::getIkSol, this);
    }

    LegKinematicTree::LegKinematicTree(int leg_id, std::string _urdf_param):
        leg_id_(leg_id),
        urdf_(_urdf_param),
        timeout_(0.005),
        eps_(2e-5)
    {
        init();
    }

    LegKinematicTree::LegKinematicTree(int leg_id, std::string _urdf_param, double _timeout, double _eps):
        leg_id_(leg_id),
        urdf_(_urdf_param),
        timeout_(_timeout),
        eps_(_eps)
    {
        init();
    }

    void LegKinematicTree::setJointPose(const std::vector<double> jnt_pose)
    {
        if(jnt_pose.size() == target_jnt_pos_.data.size())
        {
            for(int i=0;i<target_jnt_pos_.data.size();i++)
            {
                target_jnt_pos_.data[i] = jnt_pose[i];
            } 
        }
    }

    bool LegKinematicTree::getFkSol()
    {
        int rc = 0;
        rc = fk_solver_ptr_->JntToCart(target_jnt_pos_, target_ee_pos_); // calculate end effector pose
        ROS_INFO("\n%f",(float)rc);  
        if(rc < 0)
        {
            ROS_INFO("FK Not valid");
            return false;
        }
        else
        {
        return true;
        }
    }

    void LegKinematicTree::setEndPose(const std::vector<double> _pose) // x, y, z, R, P, Y
    {
        double x = _pose[0];
        double y = _pose[1];
        double z = _pose[2];
        double R = _pose[3];
        double P = _pose[4];
        double Y = _pose[5];
        KDL::Vector v(x, y, z);
        target_ee_pos_ = KDL::Frame(v);
    }

    void LegKinematicTree::setBiasedEndPose(const std::vector<double> _pose) // x, y, z, R, P, Y
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
        // KDL::Rotation::RPY(R, P, Y),
        target_ee_pos_ = KDL::Frame(v);
    }

    bool LegKinematicTree::getIkSol()
    {
        // const unsigned int num_jnts = NUM_JNTS;
        // KDL::JntArray result(num_jnts); 
        // int rc = 0; 
        // rc = ik_solver_ptr_->CartToJnt(jnt_pos_, target_ee_pos_, result);
        // ROS_INFO("\n%f",(float)rc);
        // ROS_INFO("Inverse Kinematic Angles \nJoint 1: %f\nJoint 2: %f\nJoint 3: %f\n", (float)result.data[0],(float)result.data[0],(float)result.data[0]);
        // //TODO: Formally make the "invalid bounds"
        // // if(rc < 0)
        // // {
        // //     ROS_INFO("Not Valid rc = %i",rc);
        // //     return false;
        // // }
        // // else
        // // {
        // //     // clear out the joint configuration sent and replace it with the new calculated values
        // target_jnt_pos_ = result;
        // return true;
        // // }
        /*
            KDL is slow for IK calcs. Use analytical trig solution for faster sol.
        */

        Eigen::Matrix4d h;  //transformation matrix for the joint
        double a_12 = 0.1;
        double a_23  = 0.25;
        double a_3e = 0.25;

        Eigen::Vector3d p_ee = Eigen::Vector3d(target_ee_pos_.p.data[0], target_ee_pos_.p.data[1], -target_ee_pos_.p.data[2]);    //position of the end effector
        target_jnt_pos_.data.x() = atan2(p_ee.y(),p_ee.x());    // calculate the angle of the coxa joint

        p_ee = p_ee-Eigen::Vector3d(a_12,0,0); //shift over to the femur joint
        double p_2e = pow(p_ee.x(),2)+pow(p_ee.z(),2); //calculate the hypotenuse^2 from the femur joint to end effector 
        // Transformation matrix for all joints
        double alpha = atan2(-p_ee.z(),p_ee.x());   //part of the b1 angle from the LoC
        double num = (pow(a_23,2)+p_2e-pow(a_3e,2))/(2.0*a_23*sqrt(p_2e));  //LoC for femur angle
        if(num>1){  //limit to pi
            num=1;
        }
        if(num<-1){
            num=-1;
        }
        double beta = acos(num);    //other part of the triangle
        target_jnt_pos_.data.y() = beta+alpha; //angle of the 

        num = (pow(a_3e,2)+pow(a_23,2)-p_2e)/(2.0*a_23*a_3e);
            if(num>1){  //limit to pi
            num=1;
        }
        if(num<-1){
            num=-1;
        }
        target_jnt_pos_.data.z() = acos(num)-M_PI;
        
        //Enforce Limits
        // ForceLegConstraints(jnt_pos_.data.x(),"coxa");
        // ForceLegConstraints(jnt_pos_.data.y(),"femur");
        // ForceLegConstraints(jnt_pos_.data.z(),"tibia");
    }

    bool LegKinematicTree::getIkSol(ardent_msgs_simple::SolveLegIk::Request &request, ardent_msgs_simple::SolveLegIk::Response &response)  
    {
        ardent_msgs_simple::LegStatistics target_leg_state;
        response.target_joints.clear();
        std::vector<double> jnt_pose = {(float)target_leg_state.ee_pos.position.x,
                                        (float)target_leg_state.ee_pos.position.y,
                                        (float)target_leg_state.ee_pos.position.z};                       
        setEndPose(jnt_pose);
        getIkSol();
        
    
    } 

    void LegKinematicTree::publishJointState()
    {
        sensor_msgs::JointState joint_state_;
        ROS_INFO("pub joint state");
        
        jnt_pos_ = target_jnt_pos_;
        KDL::Frame temp_frame;
        int rc = fk_solver_ptr_->JntToCart(jnt_pos_, temp_frame); // calculate end effector pose
        auto print_frame_lambda = [](KDL::Frame f)
        {
            double x, y, z, roll, pitch, yaw;
            x = f.p.x();
            y = f.p.y();
            z = f.p.z();
            f.M.GetRPY(roll, pitch, yaw);
            std::cout <<"Current Frame from FK:" "x:" << x << " y:" << y << " z:" << z << " roll:" << roll << " pitch:" << pitch << " yaw:" << yaw << std::endl;
        };
        
        print_frame_lambda(temp_frame);   
        for(int i=0; i<3;i++)
        {
            joint_state_.name.push_back(joint_name_[i]); 
            joint_state_.position.push_back(jnt_pos_.data[i]);
            // joint_state_.velocity.push_back(0.0);
            // joint_state_.effort.push_back(0.0);
            joint_state_.header.stamp = ros::Time::now();
        }
        joint_pub_.publish(joint_state_);
    }


