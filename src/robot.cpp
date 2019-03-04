#include "../include/ardent_states/robot.h"

using namespace ardent_model;

Robot::Robot(std::string robot_name) : robot_model_loader_(robot_name), 
  kinematic_model_(robot_model_loader_.getModel()),
  kinematic_state_(new robot_state::RobotState(kinematic_model_))
{
  kinematic_state_->setToDefaultValues();
  ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());

  
  num_legs_ = 5;
  
  for(int i=1;i<num_legs_;i++){
      legs_.push_back(kinematic_state_->getJointModelGroup("leg_"+std::to_string(i)));
      ROS_INFO("Leg_%s added", std::to_string(i));
  }
  //initialize the legs based on the body offset
}

Robot::~Robot()
{
  //delete kinematic_state_;
}

ros::Time RobotState::getTime()
{
    return current_time_;
}

void Robot::publishLegPosition(std::string leg_id, Eigen::Vector3d& ee_pos)
{
    // int leg_map = getMappedLeg(leg_id);
    // legs_[leg_map].getIkCommandedAngles(ee_pos);
    // legs_[leg_map].publishCommandedJointState();
}

std::string Robot::getMappedLeg(int leg_num)
{
    // static const std::map<int,std::string> leg_map{
    //     {0, "rf"},
    //     {1,"rm"},
    //     {2, "rr"},
    //     {3, "lf"},
    //     {4, "lm"},
    //     {5, "lr"}
    // };
    // return leg_map.at(leg_num);
}
int Robot::getMappedLeg(std::string leg_id)
{
    // static const std::map<std::string, int> leg_map{
    //     {"rf", 0},
    //     {"rm", 1},
    //     {"rr", 2},
    //     {"lf", 3},
    //     {"lm", 4},
    //     {"lr", 5}
    // };return leg_map.at(leg_id);

}
bool Robot::checkStability()
{
    // std::vector<float> contact_legs;
    // for(int i=0;i<num_legs_;i++){
    //     //if(GetMappedLeg(i))
    // }
}

bool Robot::initXml(const std::string xml)
{
  
}

ros::Time Robot::getTime()
{
  return ros::Time::now();
}

template <class T>
int findIndexByName(const std::vector<boost::shared_ptr<T> >& v, 
      const std::string &name)
{
  for (unsigned int i = 0; i < v.size(); ++i)
  {
    if (v[i]->name_ == name)
      return i;
  }
  return -1;
}

RobotState::RobotState(Robot *model)
  : model_(model)
{
 
}


JointState *RobotState::getJointState(const std::string &name)
{
  std::map<std::string, JointState*>::iterator it = joint_states_map_.find(name);
  if (it == joint_states_map_.end())
    return NULL;
  else
    return it->second;
}

const JointState *RobotState::getJointState(const std::string &name) const
{
  std::map<std::string, JointState*>::const_iterator it = joint_states_map_.find(name);
  if (it == joint_states_map_.end())
    return NULL;
  else
    return it->second;
}

bool RobotState::isHalted()
{
}

void RobotState::enforceSafety()
{
  for (size_t i = 0; i < model_->legs_.size(); ++i)
  {
    // model_->legs_[i].forceJointConstraints();
  }
}

void RobotState::zeroCommands()
{
  for (size_t i = 0; i < joint_states_.size(); ++i)
    joint_states_[i].commanded_effort_ = 0;
}


