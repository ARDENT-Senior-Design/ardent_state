#include "../include/ardent_state_simple/robot.h"

namespace ardent_model
{
    Robot::Robot()
    {
        if(init())
        {
            ROS_INFO("Robot Initialized");
        }
    }

    bool Robot::init() 
    {
        std::string robot_desc_string;
        // Get URDF XML
        if (!n.getParam("robot_description", robot_desc_string)) {
            ROS_FATAL("Could not load the xml from parameter: robot_description");
            return false;
        }

        // Get Root and Tip From Parameter Server
        n.param("root_name_body", root_name, std::string("body"));
        n.param("tip_name_body", tip_name, std::string("coxa"));
        n.param("clearance", default_body_height, 0.2);

        // Load and Read Models
        if (!loadModel(robot_desc_string)) {
            ROS_FATAL("Could not load models!");
            return false;
        }

        client = n.serviceClient<ardent_msgs_simple::SolveLegIk>("/get_ik");
        joints_pub = n.advertise<ardent_msgs_simple::JointState>("joints_to_controller", 1);
        body_move_sub = n.subscribe<ardent_msgs_simple::RobotStatistics>("/teleop/move_body", 1, &Robot::teleopBodyMove, this);
        body_cmd_sub = n.subscribe<ardent_msgs_simple::RobotCommand>("/teleop/body_command", 1, &Robot::teleopBodyCmd, this);

        robot.leg_radius = 0.11;
        robot.pose.position.z = -0.016;
        ros::Duration(1).sleep();
        if (calculateKinematics(&robot)){
                joints_pub.publish(legs);
        }
        ROS_INFO("Ready to receive teleop messages... ");

        return true;
    }

    bool Robot::loadModel(const std::string xml)
    {
        //Construct tree with kdl_parser
        KDL::Tree tree;
        std::string root_name = "body";
        std::string tip_name = "coxa_";

        if (!kdl_parser::treeFromString(xml, tree)) {
            ROS_ERROR("Could not initialize tree object");
            return false;
        }
        ROS_INFO("Construct tree");

        //Get coxa and leg_center frames via segments (for calculating vectors)
        std::map<std::string,KDL::TreeElement>::const_iterator segments_iter;
        std::string link_name_result;
        for (int i=1; i<num_legs_+1; i++){
            link_name_result = root_name;
            segments_iter = tree.getSegment(link_name_result);
            frames.push_back((*segments_iter).second.segment.getFrameToTip());
        }
        for (int i=1; i<num_legs_+1; i++){
            link_name_result = tip_name + std::to_string(i);
            segments_iter = tree.getSegment(link_name_result);
            frames.push_back((*segments_iter).second.segment.getFrameToTip());
        }
        ROS_INFO("Get frames");

        //Vector iterators
        for (int i=0; i<num_legs_; i++){
            frames[i] = frames[i] * frames[i+num_legs_];
        }
        frames.resize(num_legs_);

        for (int i=0; i<num_legs_; i++){
            for (int j = 0; j < num_joints; j++) {
                legs.joints_state[i].joint[j] = 0;
            }
        }

        return true;
    }

    bool Robot::callService (KDL::Vector* vector)
    {
        ardent_msgs_simple::LegStatistics leg_pos_buf;
        srv.request.leg_index.clear();
        srv.request.target_ee_state.clear();
        srv.request.current_joints.clear();

        //Creating message to request
        for (int i=0; i<num_legs_; i++){
            srv.request.leg_index.push_back(i);
            leg_pos_buf.ee_pos.position.x = vector[i].x();
            leg_pos_buf.ee_pos.position.y = vector[i].y();
            leg_pos_buf.ee_pos.position.z = vector[i].z();
            srv.request.target_ee_state.push_back(leg_pos_buf);
            srv.request.current_joints.push_back(legs.joints_state[i]);
        }
        //Call service and parsing response
        if (client.call(srv)){
            if (srv.response.error_codes==srv.response.IK_FOUND){
                for (int i=0; i<num_legs_; i++){
                    for (int j = 0; j < 3; j++) {
                            legs.joints_state[i] = srv.response.target_joints[i];
                    }
                }
            }
            else {
                ROS_ERROR("An IK solution could not be found");
                return 0;
            }
        }
        else {
            ROS_ERROR("Failed to call service");
            return 0;
        }
        return true;
    }

    bool Robot::calculateKinematics (ardent_msgs_simple::RobotStatistics* body_ptr)
    {
        //Body rotation
        rotation = KDL::Rotation::RPY(body_ptr->pose.orientation.y,body_ptr->pose.orientation.x,body_ptr->orientation.z);

        //Distance from body center to leg tip
        femur_frame = KDL::Frame (KDL::Vector (body_ptr->leg_radius,0,0));

        //Offset from center
        offset_vector_ = KDL::Vector (body_ptr->pose.position.x,body_ptr->pose.position.y,body_ptr->pose.position.z);
        rotation_correction_ = KDL::Vector (body_ptr->pose.position.z * tan(body_ptr->pose.orientation.x), -(body_ptr->pose.position.z * tan(body_ptr->pose.orientation.roll)), 0);

        for (int i=0; i<num_legs_; i++)
        {
            //Get tip frames
            tibia_foot_frame = frames[i] * femur_frame;
            //Get tip vectors with body position
            final_vector[i] = (rotation * tibia_foot_frame.p) + offset_vector_ + rotation_correction_;
            //ROS_DEBUG("Position vector leg%s\tx: %f\ty: %f\tz: %f", suffixes[i].c_str(),
            //final_vector[i](0),final_vector[i](1),final_vector[i](2));
        }

        //ROS_DEBUG("Call service: /leg_ik_service/get_ik");
        if (!callService(final_vector))
        {
            return 0;
        }

        return true;
    }

    void Robot::teleopBodyMove(const ardent_msgs_simple::RobotStatisticsConstPtr &body_state)
    {
        robot.pose.position.x = body_state->pose.position.x;
        robot.pose.position.y = body_state->pose.position.y;
        robot.pose.position.z = body_state->pose.position.z;
        robot.pose.orientation.x = body_state->pose.orientation.x;    // Pitch
        robot.pose.orientation.y = body_state->pose.orientation.y;    // Roll
        robot.pose.orientation.z = body_state->pose.orientation.z;   // Yaw
        robot.leg_radius = body_state->leg_radius;
        if (calculateKinematics(&robot))
        {
            // joints_pub.publish(legs);
        }
    }

    void Robot::teleopBodyCmd(const ardent_msgs_simple::RobotCommandConstPtr &body_cmd)
    {
        if (body_cmd->cmd == body_cmd->STAND_UP_CMD){
            ROS_ERROR("STAND_UP_CMD");
            ros::Rate r(25);
            while (robot.pose.position.z >= -default_body_height){
            robot.pose.position.z -= 0.0025;
            r.sleep();
            if (calculateKinematics(&robot)){
                    joints_pub.publish(legs);
            }
            }
        }
        if (body_cmd->cmd == body_cmd->SEAT_DOWN_CMD){
            ROS_ERROR("SEAT_DOWN_CMD");
            ros::Rate r(25);
            while (robot.pose.position.z <= -0.016){
                robot.pose.position.z += 0.0025;
                r.sleep();
                if (calculateKinematics(&robot)){
                        joints_pub.publish(legs);
                }
            }
        }
    } 
}