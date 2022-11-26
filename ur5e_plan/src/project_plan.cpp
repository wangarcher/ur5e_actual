// a simple planning demo, initialized in 210813
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <mission_mode/mode_switch.h>

class GeneralPickPlace
{
public:
    Eigen::Vector3d cam_obj_translation_;
    Eigen::Quaterniond cam_obj_orientation_;
    bool first_reco_flag;    // outwards flag, when the ee reached the desired pre-picking place
    bool second_reco_flag;   // outwards flag, when the ee reached the desired pre-placeing place
    bool pick_flag = false;  // inwards flag, when the flag set as true, the robot is ready for the pick motion sequence.
    bool place_flag = false; // inwards flag, whent the flag set as true, the robot is ready for the place motion sequence.

public:
    void obj_visual_callback(const geometry_msgs::PoseStamped &odom);
};

void GeneralPickPlace::obj_visual_callback(const geometry_msgs::PoseStamped &odom)
{
    cam_obj_translation_ << odom.pose.position.x,
        odom.pose.position.y,
        odom.pose.position.z;
    cam_obj_orientation_.coeffs() << odom.pose.orientation.x,
        odom.pose.orientation.y,
        odom.pose.orientation.z,
        odom.pose.orientation.w;
    if (odom.header.frame_id == "1st")
    {
        pick_flag = true;
        place_flag = false;
    }
    if (odom.header.frame_id == "2nd")
    {
        place_flag = true;
        pick_flag = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "project_plan_pipeline");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(7);
    spinner.start();

    // some flag were set here, so as to make sure the sequence flow running in a proper way.
    bool pre_pick_flag = true;
    bool pick_flag = false;
    bool pre_place_flag = false;
    bool place_flag = false;
    bool sequence_flag = true;

    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    arm.allowReplanning(true); // Evangelion 3.33: You Can(Not) Redo.
    arm.setGoalJointTolerance(0.001);
    arm.setGoalPositionTolerance(0.001);      // set position torlerance
    arm.setGoalOrientationTolerance(0.01);    // set orientation torlerance
    arm.setMaxAccelerationScalingFactor(0.2); // set max accelerations
    arm.setMaxVelocityScalingFactor(0.2);

    std::string end_effector_link = arm.getEndEffectorLink(); // get the end_effector link
    std::cout << "end_effector_link: " << end_effector_link << std::endl;
    std::string reference_frame = "base_link"; // set the reference frame
    arm.setPoseReferenceFrame(reference_frame);

    GeneralPickPlace general_pick_place;

    ros::Rate loop_rate(10);

    ros::Subscriber sub = n.subscribe("/object_visual_odometry", 1000, &GeneralPickPlace::obj_visual_callback, &general_pick_place);
    ros::ServiceClient ur_dashboard_client = n.serviceClient<mission_mode::mode_switch>("/arm_mode_switch");
    ros::Publisher reco_trigger_pub = n.advertise<std_msgs::Int8>("/reco_trigger", 1);

    // the messages may needed in the process
    std_msgs::Int8 reco_trigger_msg;
    mission_mode::mode_switch open_gripper_sig;
    mission_mode::mode_switch close_gripper_sig;
    mission_mode::mode_switch arm_working_sig;
    arm_working_sig.request.mode_flag = 1;
    open_gripper_sig.request.mode_flag = 7;
    close_gripper_sig.request.mode_flag = 8;

    // geometry info
    Eigen::Quaterniond tool_tcp_orientation(1, 0, 0, 0);
    //<<<<<<< HEAD
    Eigen::Vector3d tool_tcp_translation(0, 0, -0.205);                                                        // after calibration
    Eigen::Quaterniond tool_cam_orientatoin(0.494830699069, 0.489293676245, -0.504172738262, 0.511413851629); // after calibration
    Eigen::Vector3d tool_cam_translation(0.00100323541413, -0.0877033481828, 0.119296843695);                 // after calibration
    Eigen::Quaterniond bias(0, 0.707, 0, -0.707);
    //=======
    // Eigen::Vector3d tool_tcp_translation(0, 0, -0.20); //tool0 to tcp
    // Eigen::Quaterniond tool_cam_orientatoin(0.707, 0, 0, 0.707);
    // Eigen::Vector3d tool_cam_translation(0.07, 0.05, -0.02); //tool0 to cam
    //>>>>>>> 92f8bf9f96270783992d3805f06be4b59e0be0ad

    Eigen::Quaterniond base_tool_orientation;
    Eigen::Vector3d base_tool_translation;
    Eigen::Quaterniond base_cam_orientation;
    Eigen::Vector3d base_cam_translation;
    Eigen::Quaterniond base_obj_orientation;
    Eigen::Vector3d base_obj_translation;
    Eigen::Quaterniond des_base_tool_orientation;
    Eigen::Vector3d des_base_tool_translation;

    while (n.ok() && sequence_flag)
    {
        if (pre_pick_flag)
        {
            std::cout << "Caution: start the ur connect driver 1!" << std::endl;
            ur_dashboard_client.call(arm_working_sig);
            sleep(5);

            std::cout << "Caution: moving to the pre-pick pose!" << std::endl;
            arm.setNamedTarget("1st_reco_start"); // first reco state, for the best top-looking view
            arm.move();
            sleep(1);

            ur_dashboard_client.call(open_gripper_sig);
            sleep(7);

            std::cout << "Caution: start the ur connect driver 2!" << std::endl;
            ur_dashboard_client.call(arm_working_sig);
            sleep(5);
            reco_trigger_msg.data = 1;
            reco_trigger_pub.publish(reco_trigger_msg);
            sleep(1);
            pre_pick_flag = false;
            pick_flag = true;
        }

        if (pick_flag && general_pick_place.pick_flag)
        {
            std::cout << "Caution: object pose received, now picking!" << std::endl;
            geometry_msgs::Pose now_pose = arm.getCurrentPose(end_effector_link).pose;
            std::cout << "now tool position: [x,y,z]: ["
                      << now_pose.position.x << "," << now_pose.position.y << "," << now_pose.position.z << "]"
                      << std::endl;
            std::cout << "now tool orientation: [x,y,z,w]: ["
                      << now_pose.orientation.x << "," << now_pose.orientation.y << "," << now_pose.orientation.z << "," << now_pose.orientation.w << "]"
                      << std::endl;

            base_tool_translation << now_pose.position.x, now_pose.position.y, now_pose.position.z;
            base_tool_orientation.coeffs() << now_pose.orientation.x,
                now_pose.orientation.y,
                now_pose.orientation.z,
                now_pose.orientation.w;

            base_cam_orientation = base_tool_orientation * tool_cam_orientatoin;
            base_cam_translation = base_tool_orientation.toRotationMatrix() * tool_cam_translation + base_tool_translation;

            std::cout << "now cam position: [x,y,z]: [" << base_cam_translation.transpose() << std::endl;
            std::cout << "now cam orientation: [x, y, z, w]: ["
                      << base_cam_orientation.x() << ", "
                      << base_cam_orientation.y() << ", "
                      << base_cam_orientation.z() << ", "
                      << base_cam_orientation.w() << "]" << std::endl;

            // base_obj_orientation = base_cam_orientation * general_pick_place.cam_obj_orientation_;
            base_obj_orientation = base_cam_orientation * general_pick_place.cam_obj_orientation_ * bias;
            base_obj_translation = base_cam_orientation.toRotationMatrix() * general_pick_place.cam_obj_translation_ + base_cam_translation;

            std::cout << "now obj position: [x,y,z]: [" << base_obj_translation.transpose() << std::endl;
            std::cout << "now obj orientation: [x, y, z, w]: ["
                      << base_obj_orientation.x() << ", "
                      << base_obj_orientation.y() << ", "
                      << base_obj_orientation.z() << ", "
                      << base_obj_orientation.w() << "]" << std::endl;

            des_base_tool_orientation = base_obj_orientation * tool_tcp_orientation;
            des_base_tool_translation = base_obj_orientation.toRotationMatrix() * tool_tcp_translation + base_obj_translation;

            std::cout << "des tool position: [x,y,z]: [" << des_base_tool_translation.transpose() << std::endl;
            std::cout << "des tool orientation: [x, y, z, w]: ["
                      << des_base_tool_orientation.x() << ", "
                      << des_base_tool_orientation.y() << ", "
                      << des_base_tool_orientation.z() << ", "
                      << des_base_tool_orientation.w() << "]" << std::endl;

            std::vector<geometry_msgs::Pose> waypoints;
            geometry_msgs::Pose pose1;
            pose1.position.x = des_base_tool_translation(0);
            pose1.position.y = des_base_tool_translation(1);
            pose1.position.z = base_tool_translation(2);
            pose1.orientation.x = base_obj_orientation.x();
            pose1.orientation.y = base_obj_orientation.y();
            pose1.orientation.z = base_obj_orientation.z();
            pose1.orientation.w = base_obj_orientation.w();
            waypoints.push_back(pose1);

            geometry_msgs::Pose pose2;
            pose2.position.x = des_base_tool_translation(0);
            pose2.position.y = des_base_tool_translation(1);
            pose2.position.z = des_base_tool_translation(2);
            pose2.orientation.x = base_obj_orientation.x();
            pose2.orientation.y = base_obj_orientation.y();
            pose2.orientation.z = base_obj_orientation.z();
            pose2.orientation.w = base_obj_orientation.w();
            waypoints.push_back(pose2);

            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.002;
            double fraction = 0.0;
            int maxtries = 100; // max attempts
            int attempts = 0;   // attempts done

            while (fraction < 1.0 && attempts < maxtries)
            {
                fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
                attempts++;
                if (attempts % 10 == 0)
                    ROS_INFO("Still trying after %d attempts...", attempts);
            }

            if (fraction == 1)
            {
                ROS_INFO("Path computed successfully. Moving the arm.");
                // plan data
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                // execute
                arm.execute(plan);
                sleep(1);
            }
            else
            {
                ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
            }

            std::cout << "Caution: now picking!" << std::endl;
            ur_dashboard_client.call(close_gripper_sig);
            sleep(7);

            std::cout << "Caution: start the ur connect driver 3!" << std::endl;
            ur_dashboard_client.call(arm_working_sig);
            sleep(3);
            pick_flag = false;
            pre_place_flag = true;
        }

        if (pre_place_flag)
        {
            std::cout << "Caution: moving to the pre-place pose!" << std::endl;
            arm.setNamedTarget("2nd_reco_start"); // second reco state, for the best top-looking view
            arm.move();
            sleep(1);

            reco_trigger_msg.data = 2;
            reco_trigger_pub.publish(reco_trigger_msg);
            sleep(1);
            pre_place_flag = false;
            place_flag = true;
        }

        if (place_flag && general_pick_place.place_flag)
        {
            std::cout << "Caution: workspace pose received, now placing!" << std::endl;
            geometry_msgs::Pose now_pose = arm.getCurrentPose(end_effector_link).pose;
            std::cout << "now Robot position: [x,y,z]: ["
                      << now_pose.position.x << "," << now_pose.position.y << "," << now_pose.position.z << "]"
                      << std::endl;
            std::cout << "now Robot orientation: [x,y,z,w]: ["
                      << now_pose.orientation.x << "," << now_pose.orientation.y << "," << now_pose.orientation.z << "," << now_pose.orientation.w << "]"
                      << std::endl;

            base_tool_translation << now_pose.position.x, now_pose.position.y, now_pose.position.z;
            base_tool_orientation.coeffs() << now_pose.orientation.x,
                now_pose.orientation.y,
                now_pose.orientation.z,
                now_pose.orientation.w;

            base_cam_orientation = base_tool_orientation * tool_cam_orientatoin;
            base_cam_translation = base_tool_orientation.toRotationMatrix() * tool_cam_translation + base_tool_translation;

            base_obj_orientation = base_cam_orientation * general_pick_place.cam_obj_orientation_ * bias;
            base_obj_translation = base_cam_orientation.toRotationMatrix() * general_pick_place.cam_obj_translation_ + base_cam_translation;

            des_base_tool_orientation = base_obj_orientation * tool_tcp_orientation;
            des_base_tool_translation = base_obj_orientation.toRotationMatrix() * tool_tcp_translation + base_obj_translation;

            std::cout << "now obj position: " << des_base_tool_translation.transpose() << std::endl;
            std::cout << "now obj orientation: [x, y, z, w]: ["
                      << des_base_tool_orientation.x() << ", "
                      << des_base_tool_orientation.y() << ", "
                      << des_base_tool_orientation.z() << ", "
                      << des_base_tool_orientation.w() << "]" << std::endl;

            std::vector<geometry_msgs::Pose> waypoints;
            geometry_msgs::Pose pose1;
            pose1.position.x = des_base_tool_translation(0);
            pose1.position.y = des_base_tool_translation(1);
            pose1.position.z = base_tool_translation(2);
            pose1.orientation.x = base_obj_orientation.x();
            pose1.orientation.y = base_obj_orientation.y();
            pose1.orientation.z = base_obj_orientation.z();
            pose1.orientation.w = base_obj_orientation.w();
            waypoints.push_back(pose1);

            geometry_msgs::Pose pose2;
            pose2.position.x = des_base_tool_translation(0);
            pose2.position.y = des_base_tool_translation(1);
            pose2.position.z = des_base_tool_translation(2);
            pose2.orientation.x = base_obj_orientation.x();
            pose2.orientation.y = base_obj_orientation.y();
            pose2.orientation.z = base_obj_orientation.z();
            pose2.orientation.w = base_obj_orientation.w();
            waypoints.push_back(pose2);

            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.002;
            double fraction = 0.0;
            int maxtries = 100; // max attempts
            int attempts = 0;   // attempts done

            while (fraction < 1.0 && attempts < maxtries)
            {
                fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
                attempts++;
                if (attempts % 10 == 0)
                    ROS_INFO("Still trying after %d attempts...", attempts);
            }

            if (fraction == 1)
            {
                ROS_INFO("Path computed successfully. Moving the arm.");
                // plan data
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                // execute
                arm.execute(plan);
                sleep(1);
            }
            else
            {
                ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
            }
            ur_dashboard_client.call(open_gripper_sig);
            sleep(7);
            place_flag = false;
            sequence_flag = false;
        }
        loop_rate.sleep();
    }

    ur_dashboard_client.call(arm_working_sig);
    sleep(3);
    std::vector<geometry_msgs::Pose> waypoints;

    std::cout << "Caution: moving to the end pose!" << std::endl;
    arm.setNamedTarget("2nd_reco_start"); // second reco state, for the best top-looking view
    arm.move();
    sleep(7);
    std::cout << "Info: mission accomplished!!!" << std::endl;
    ros::waitForShutdown();

    return 0;
}
