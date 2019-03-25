
            /// Get the time when the current controller cycle was started
            // ros::Time getTime();
#include "kinematic_tree.h"
#include "tf/transform_broadcaster.h"


namespace ardent_model {

    #define NUM_LEGS 4
    class Robot{
        public:
            Robot();
            bool init();

        private:
            ros::NodeHandle n;
            // KDL::Frame body_frame_;
            std::vector<LegKinematicTree> legs_;
            KDL::Rotation rotation_;
            const static unsigned int num_legs_ = NUM_LEGS;
            const std::string urdf_;
            KDL::Vector offset_vector_, rotation_correction, final_vector[num_legs_];
            

            tf::TransformBroadcaster odom_broadcaster;

            // bool calculateKinematics (crab_msgs::BodyState* body_ptr);
            // bool callService (KDL::Vector* vector);
            // void teleopBodyMove (const crab_msgs::BodyStateConstPtr &body_state);
            // void teleopBodyCmd (const crab_msgs::BodyCommandConstPtr &body_cmd);
    };
}