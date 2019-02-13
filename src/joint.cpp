/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "../include/ardent_states/joint.h"
#include <map>
#include <string>
#include <vector>  
#include <cfloat>
#include <limits> // for std::numeric_limits

using namespace std;
using namespace ardent_model;

JointState::JointState(std::string joint_id) : commanded_effort_(0.0), 
  calibrated_(true), reference_position_(0.0)
{
  id_ = joint_id;
  measured_position_ = 0.0; 
  measured_velocity_ = 0.0; 
  measured_acceleration_ = 0.0;
  measured_effort_ = 0.0;
}


void JointState::constrain()
{
  commanded_effort_ = abs(commanded_effort_)>max_abs_effort_ ? max_abs_effort_*sgn(commanded_effort_) : commanded_effort_;
  commanded_velocity_ = abs(commanded_velocity_)>max_abs_velocity_ ? max_abs_velocity_*sgn(commanded_velocity_) : commanded_effort_;
  commanded_position_ = commanded_position_>max_position_ ? max_position_ : commanded_position_;
  commanded_position_ = commanded_position_<min_position_ ? min_position_ : commanded_position_;
}

void JointState::update() 
{
  // TODO: Fill these up with the topics that they are listening in to.
  // ros::Subscriber joint_sub;
  measured_position_ = 0.0;
  measured_velocity_ = 0.0;
  measured_effort_ = 0.0;
  calibrated_ = 0;
}

template <class T> inline int JointState::sgn(T v) 
{
    return (v > T(0)) - (v < T(0));
};