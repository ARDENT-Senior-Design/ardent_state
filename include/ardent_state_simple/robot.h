#ifndef ROBOT_H_
#define ROBOT_H_
            /// Get the time when the current controller cycle was started
            // ros::Time getTime();
#include "leg_kinematic_tree.h"
#include <tf/transform_broadcaster.h>
#include <ardent_msgs_simple/RobotStatistics.h>
#include <ardent_msgs_simple/RobotCommand.h>
#include <ardent_msgs_simple/SolveLegIk.h>
#include <kdl_parser/kdl_parser.hpp>

namespace ardent_model {

    #define NUM_LEGS 4
    
    class Robot{
        public:
            Robot();
            bool init();

        private:
            ros::NodeHandle n;
		    std::vector<KDL::Frame> frames; // Frames for each coxa location
            
            ardent_msgs_simple::RobotStatistics robot;
            ardent_msgs_simple::LegStatistics legs;
            ardent_msgs_simple::SolveLegIk srv;

            const static unsigned int num_legs_ = NUM_LEGS;
            KDL::Rotation rotation_;
            KDL::Vector offset_vector_, rotation_correction_, final_vector[num_legs_];
            std::string root_name, tip_name;
            ros::ServiceClient client;
            ros::Subscriber body_move_sub;
            ros::Subscriber body_cmd_sub;

            tf::TransformBroadcaster odom_broadcaster;
            
            int default_body_height;

            bool loadModel(const std::string xml);
            bool calculateKinematics (ardent_msgs_simple::RobotStatistics* body_ptr);
            bool callService (KDL::Vector* vector);
            void teleopBodyMove (const ardent_msgs_simple::RobotStatisticsConstPtr &body_state);
            void teleopBodyCmd (const ardent_msgs_simple::RobotCommandConstPtr &body_cmd);
    };
}

#endif