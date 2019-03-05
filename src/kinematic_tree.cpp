#include "../include/ardent_states/kinematic_tree.h"

using namespace ardent_model;

LegKinematics::LegKinematics(robot_model::RobotModelPtr kinematic_model, std::string id) : 
  kinematic_state_(new robot_state::RobotState(kinematic_model)) , n("leg_"+id)
{
  id_ = id;
  kinematic_state_->setToDefaultValues();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str()); 
  joint_model_group_ = kinematic_state_->getJointModelGroup("leg_"+id_);
  joint_names_ = joint_model_group_->getVariableNames();
  
  kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_angles_);
  // ROS_INFO("Leg_"+id_+" added");
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names_[i].c_str(), joint_angles_[i]);
  }
  
}

LegKinematics::~LegKinematics()
{

}

void LegKinematics::getJointAngles()
{
  kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_angles_);
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names_[i].c_str(), joint_angles_[i]);
  }
}

void LegKinematics::setJointAngles()
{
  kinematic_state_->setJointGroupPositions(joint_model_group_, joint_angles_);
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names_[i].c_str(), joint_angles_[i]);
  }
}
// LegKinematics::getJointAngles()
// {
//   kinematic_state_->copyJointGroupPositions(joint_model_group, joint_values);
//   for (std::size_t i = 0; i < joint_names.size(); ++i)
//   {
//     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
//   }
// }





// BodyKinematics::BodyKinematics() 
//     : body_pose(Vector3d(0, 0, BODY_THICKNESS)), radius(0.2)
// {
// }

// Eigen::Vector3d BodyKinematics::getRPY()
// {
    
// }

// Eigen::Matrix4d BodyKinematics::getLegPosition(std::string leg_id)
// {
//     double t_d = BodyKinematics::getLegAngleOffset(leg_id);
//     double t_y = 0; // yaw of the robot
//     double t_r = 0; // roll of the robot
//     double t_p = 0; // pitch of the robot

//     Eigen::Matrix4d Tz; // translation about the robot z-axis
//     Tz << cos(t_d+t_y), -sin(t_d+t_y), 0, radius*cos(t_d), sin(t_d+t_y), cos(t_d+t_y), 0, radius*sin(t_d), 0, 0, 1, 0, 0, 0, 0, 1;

//     Eigen::Matrix4d Tx;
//     Tx << 1, 0, 0, radius, 0, cos(t_r), -sin(t_r), 0, 0, sin(t_r), cos(t_r), 0, 0, 0, 0, 1;

//     Eigen::Matrix4d Ty;
//     Ty << cos(t_p), 0, sin(t_p), 0, 0, 1, 0, 0, -sin(t_p), 0, cos(t_p), 0, 0, 0, 0, 1;

//     return (Tx*Ty*Tz).eval();
// }

// double BodyKinematics::getLegAngleOffset(std::string leg_id)
// {
//     if(leg_id == "r1"){
//         return 1.0472;
//     }
//     else if(leg_id == "r2"){
//         return 0;
//     }
//     else if(leg_id == "r3"){
//         return -1.0472;
//     }
//     else if(leg_id == "l4"){
//         return 2.0944;
//     }
//     else if(leg_id == "l5"){
//         return 3.14;
//     }
//     else if(leg_id == "l6"){
//         return 4.128879;
//     }
//     else{
//         return 0;
//     }
// }


// LegKinematics::LegKinematics(std::string new_leg_id)
//     : coxa_length(0.1), femur_length(0.25), tibia_length(0.25)
// {
//     leg_id = new_leg_id;
//     joints_.push_back(new JointState("j_coxa_"));
//     joints_.push_back(new JointState("j_femur_"));
//     joints_.push_back(new JointState("j_tibia_"));
//     coxa_pub = nh.advertise<std_msgs::Float64>("/"+joints_[0]->id_+leg_id+"_position_controller/command",1000);
//     femur_pub = nh.advertise<std_msgs::Float64>("/"+joints_[1]->id_+leg_id+"_position_controller/command",1000);
//     tibia_pub = nh.advertise<std_msgs::Float64>("/"+joints_[2]->id_+leg_id+"_position_controller/command",1000);
//     contact_state_pub = nh.advertise<std_msgs::Bool>("/"+leg_id+"_contact/state",1000);
// }

// void LegKinematics::publishCommandedJointState()
// {   
//     ROS_DEBUG_STREAM("Joint Angles are getting published");
//     std_msgs::Float64 coxa_msg;
//     std_msgs::Float64 femur_msg;
//     std_msgs::Float64 tibia_msg;
//     if(leg_id == "l4" || leg_id == "l5" || leg_id =="l6"){
//         coxa_msg.data = -(float)joints_[0]->commanded_position_;
//     }
//     else{
//         coxa_msg.data = (float)joints_[0]->commanded_position_;
//     }
//     femur_msg.data = (float)joints_[1]->commanded_position_;
//     tibia_msg.data = (float)joints_[2]->commanded_position_;

//     coxa_pub.publish(coxa_msg);
//     femur_pub.publish(femur_msg);
//     tibia_pub.publish(tibia_msg);

//     // add velocity and acceleration down here later
// }

// void LegKinematics::getIkCommandedAngles(Vector3d& ee_pos) 
// {   
//     // local numbers to make it easier to do math. 
//     double a_12 = coxa_length;
//     double a_23  = femur_length;
//     double a_3e = tibia_length;

//     Eigen::Vector3d p_ee = Eigen::Vector3d(ee_pos.x(), ee_pos.y(), -ee_pos.z());    //position of the end effector
//     joints_[0]->commanded_position_ = atan2(p_ee.y(),p_ee.x());    // calculate the angle of the coxa joint

//     p_ee = p_ee-Vector3d(a_12,0,0); //shift over to the femur joint
//     double p_2e = pow(p_ee.x(),2)+pow(p_ee.z(),2); //calculate the hypotenuse^2 from the femur joint to end effector 
//     // Transformation matrix for all joints
//     double alpha = atan2(-p_ee.z(),p_ee.x());   //part of the b1 angle from the LoC
//     double num = (pow(a_23,2)+p_2e-pow(a_3e,2))/(2.0*a_23*sqrt(p_2e));  //LoC for femur angle
//     if(num>1){  //limit to pi
//         num=1;
//     }
//     if(num<-1){
//         num=-1;
//     }
//     double beta = acos(num);    //other part of the triangle
//     joints_[0]->commanded_position_ = beta+alpha; //angle of the 

//     num = (pow(a_3e,2)+pow(a_23,2)-p_2e)/(2.0*a_23*a_3e);
//         if(num>1){  //limit to pi
//         num=1;
//     }
//     if(num<-1){
//         num=-1;
//     }
//     joints_[1]->commanded_position_ = acos(num)-M_PI;
    
//     //Enforce Limits
//     forceJointConstraints();
// }

// void LegKinematics::getCartesianEndEffectorPosition()
// {
//     double a_12 = coxa_length;
//     double a_23  = femur_length;
//     double a_3e = tibia_length;
//     double t = joints_[0]->measured_position_; // coxa theta relative to body 
//     double b1 = joints_[1]->measured_position_; // femur theta relative to coxa
//     double g = joints_[2]->measured_position_; //gamma angle for the tibia relative to the femur
//     Eigen::Matrix4d T_01;   //transformation about coxa joint in relative z-axis
//     T_01 << cos(t), 0, sin(t), a_12*cos(t), 
//             sin(t), 0, -cos(t), a_12*sin(t), 
//             0, 1, 0, 0, 
//             0, 0, 0, 1;   

//     Eigen::Matrix4d T_12;   //transformation about the femur joint in relative z-axis
//     T_12 << cos(b1), -sin(b1), 0, a_23*cos(b1),
//             sin(b1), cos(b1), 0, a_23*sin(b1),
//             0, 0, 1, 0,
//             0, 0, 0, 1;

//     Eigen::Matrix4d T_23;   //transformation about the tibia joint in the relative z-axis
//     T_23 << cos(g), -sin(g), 0, a_3e*cos(g),
//             sin(g), cos(g), 0, a_3e*sin(g),
//             0, 0, 1, 0,
//             0, 0, 0, 1;
    
    
//     Eigen::Matrix4d T_03 = (T_01*T_12*T_23).eval(); //transform from the coxa joint to the tibia
//     Eigen::Vector4d p_ee = Eigen::Vector4d(0, 0, 0, 1);  //position of the leg end-effector relative to the tibia joint

//     Eigen::Vector4d p_0ee = (T_03*p_ee).eval();  //position of the end effector relative to the coxa joint
//     ee_pos = Eigen::Vector3d(p_0ee.x(),p_0ee.y(),p_0ee.z());
// } 

// void LegKinematics::forceJointConstraints()
// {   
//     // if leg position is farther than xy-distance, constrain the joints as well
//     for(size_t i=0;i<joints_.size();++i)
//         joints_[i]->constrain();
// }