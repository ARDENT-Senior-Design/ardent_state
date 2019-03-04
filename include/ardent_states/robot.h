#ifndef ROBOT_H_
#define ROBOT_H_

#include <ros/ros.h>
#include "kinematic_tree.h"
#include <urdf/model.h>
#include <map>
#include <string>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_parser.h>
#include <control_toolbox/filters.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


#include <sensor_msgs/JointState.h>
#include <ardent_msgs/RobotStatistics.h>
#include <ardent_msgs/LegStatistics.h>
#include <ardent_msgs/RobotCommand.h>
#include "joint.h"
#include <tinyxml.h>
#include <boost/scoped_ptr.hpp>
#include <eigen3/Eigen/Dense>

namespace ardent_model {

    class Robot{
        // Purpose of this class is to combine the leg and body kinematics into a single robot
        private: 
        public:
            ros::NodeHandle n;
            BodyKinematics body_;
            std::vector<const robot_state::JointModelGroup*> legs_;
            std::vector<ardent_msgs::LegStatisticsConstPtr> leg_state_;
            
            robot_model_loader::RobotModelLoader robot_model_loader_;
            robot_model::RobotModelPtr kinematic_model_;
            robot_state::RobotStatePtr kinematic_state_;
            int num_legs_;

            /// a pointer to the ardent hardware interface. 
            urdf::Model robot_model_;
            std::string robot_urdf_;
            
            ros::Subscriber robot_cmd;
            ros::Subscriber robot_motion;

            /** 
             * Supporting Library
             * @brief Creates a new robot
             * @param legs_ an array of the legs that will be added to the robot in order of right/left (r/l) and front (f), mid(m), and rear(r)
             */
            Robot(std::string robot_name);
            ~Robot();

            /// Initialize the robot model from an xml file. 
            // Probably faster in real time to use TiXmlElement
            bool initXml(const std::string xml);
            
            int getTransmissionIndex(const std::string &name) const;

            /// get an actuator pointer based on the actuator name. Returns NULL on failure
            // ardent_hardware_interface::Actuator* getActuator(const std::string &name) const;

            /// Get the time when the current controller cycle was started
            ros::Time getTime();

            void publishLegPosition(std::string leg_id, Eigen::Vector3d& ee_pos); //TODO: Change to include pose, vel
            void teleopRobotMove(const ardent_msgs::RobotStatisticsConstPtr &robot_state);
            void teleopRobotCommand(const ardent_msgs::RobotCommandConstPtr &robot_cmd);
            std::string getMappedLeg(int leg_num);
            int getMappedLeg(std::string leg_id);

            bool checkStability();
    }; 

    /** \brief This class provides the controllers with an interface to the robot state
     *
     * Most controllers that need the robot state should use the joint states, to get
     * access to the joint position/velocity/effort, and to command the effort a joint
     * should apply. Controllers can get access to the hard realtime clock through getTime()
     *
     * Some specialized controllers (such as the calibration controllers) can get access
     * to actuator states, and transmission states.
     */
    class RobotState 
    {
        // This is for handling the robot as a whole in the grand schemee of the world frame and trajectories
        private:
        public:

        std::map<std::string, JointState*> joint_states_map_;
        ardent_msgs::RobotStatistics robot_state_;
        // time since starting the robot 
        ros::Time current_time_;
        /// constructor
        RobotState(Robot *model);
        /// The robot model containing the transmissions, urdf robot model, and hardware interface
        Robot *model_;
        ros::Time getTime();
        /// The vector of joint states, in no particular order
        std::vector<JointState> joint_states_;
        /// Get a joint state by name
        JointState *getJointState(const std::string &name);
        /// Get a const joint state by name
        const JointState *getJointState(const std::string &name) const;

        /// Modify the commanded_effort_ of each joint state so that the joint limits are satisfied
        void enforceSafety();

        /// Checks if one (or more) of the motors are halted.
        bool isHalted();

        /// Set the commanded_effort_ of each joint state to zero
        void zeroCommands();
    };

}



#endif