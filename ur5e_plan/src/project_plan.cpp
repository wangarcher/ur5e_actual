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
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <mission_mode/mode_switch.h>



class Listener
{
public:
    Eigen::Vector3d obj_cam_translation_;
    Eigen::Quaterniond obj_cam_orientation_;
    bool stable_flag;
    bool place_flag;
    
public:
    void obj_callback(const nav_msgs::Odometry& odom);
    void signal_callback(const std_msgs::Bool& msg);
    bool placeCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
};

void Listener::obj_callback(const nav_msgs::Odometry& odom)
{

    obj_cam_translation_ << odom.pose.pose.position.x,
                            odom.pose.pose.position.y,
                            odom.pose.pose.position.z;
    obj_cam_orientation_.coeffs() <<  odom.pose.pose.orientation.x,
                                      odom.pose.pose.orientation.y,
                                      odom.pose.pose.orientation.z,
                                      odom.pose.pose.orientation.w;
}

void Listener::signal_callback(const std_msgs::Bool& msg)
{
    if (msg.data == true)
    {
        stable_flag = true;
    }
}

bool Listener::placeCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    place_flag = true;
}


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "project_plan");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(7);
	spinner.start();

    bool qr_flag = true;
    bool grip_flag = false;
    bool back_flag = false;
    bool done_flag = true;
    float grip_force = 0;
    float grip_position = 0;
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    
    arm.allowReplanning(true); // Evangelion 3.33: You Can(Not) Redo.
    arm.setGoalJointTolerance(0.001);
    arm.setGoalPositionTolerance(0.001); // set position torlerance
    arm.setGoalOrientationTolerance(0.01);  // set orientation torlerance
    arm.setMaxAccelerationScalingFactor(0.2); // set max accelerations
    arm.setMaxVelocityScalingFactor(0.2);
    
    Listener listener;

    ros::Rate loop_rate(10);

    ros::Subscriber sub2 = n.subscribe("/planner_grasp/odometry", 1000, &Listener::obj_callback, &listener);
    ros::Subscriber sub3 = n.subscribe("/planner_grasp/signal", 100, &Listener::signal_callback, &listener);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/grip_control", 1);
    ros::Publisher detection_trigger_pub = n.advertise<std_msgs::Bool>("/planner_grasp/trigger", 1);
    ros::ServiceClient launch_nav_client = n.serviceClient<std_srvs::Trigger>("/launch_nav");
    ros::ServiceClient vnc_client = n.serviceClient<mission_mode::mode_switch>("/arm_mode_switch");
    ros::ServiceServer place_server = n.advertiseService("/place_service", &Listener::placeCallback, &listener);    
    
    listener.stable_flag = false;
    listener.place_flag = false;
     mission_mode::mode_switch init_sig;
     init_sig.request.mode_flag = 1;
     vnc_client.call(init_sig);
    sleep(8);
    while(n.ok() && done_flag)
    {
        // qr_flag = false;
        // grip_flag = false;
        // back_flag = false;
        if(qr_flag)
        {
            grip_force = 0.7;
            grip_position = 0.95;
            geometry_msgs::Twist msgOut;
            msgOut.linear.x = grip_force;
            msgOut.linear.y = grip_position;
            pub.publish(msgOut);
            sleep(1);
            std::cout<<"in the frist plan"<<std::endl;
            qr_flag = false;
            grip_flag = true;
            arm.setNamedTarget("qr_start"); //qr_state, for the best overlook view
            arm.move();
        }
        if(grip_flag)
        {   
            std_msgs::Bool trigger;
            trigger.data = true;
            detection_trigger_pub.publish(trigger);
            // grip_flag = false;
            sleep(1);
            if(listener.stable_flag)
            {
                std::string end_effector_link = arm.getEndEffectorLink(); //get the end_effector link
                std::cout<<"end_effector_link: "<<end_effector_link<<std::endl;
            
                std::string reference_frame = "base_link"; // set the reference frame
                arm.setPoseReferenceFrame(reference_frame);
                arm.allowReplanning(true); // Evangelion 3.33: You Can(Not) Redo.
                arm.setGoalJointTolerance(0.001);
                arm.setGoalPositionTolerance(0.001); // set position torlerance
                arm.setGoalOrientationTolerance(0.01);  // set orientation torlerance
                arm.setMaxAccelerationScalingFactor(0.2); // set max accelerations
                arm.setMaxVelocityScalingFactor(0.2);

                geometry_msgs::Pose now_pose = arm.getCurrentPose(end_effector_link).pose;
                std::cout<<"now Robot position: [x,y,z]: ["
                        <<now_pose.position.x<<","<<now_pose.position.y<<","<<now_pose.position.z<<"]"
                        <<std::endl;
                std::cout<<"now Robot orientation: [x,y,z,w]: ["
                        <<now_pose.orientation.x<<","<<now_pose.orientation.y<<","<<now_pose.orientation.z<<","<<now_pose.orientation.w<<"]"
                        <<std::endl;

                Eigen::Quaterniond tcp_obj_orientation(1, 0, 0, 0);
                Eigen::Vector3d tcp_obj_translation(0, 0, -0.22);
                Eigen::Quaterniond cam_tool_orientatoin(1, 0, 0, 0);
                Eigen::Vector3d cam_tool_translation(-0.035, -0.09, 0.035);

                Eigen::Vector3d tool_base_translation;  
                Eigen::Quaterniond tool_base_orientation;
                tool_base_translation << now_pose.position.x, now_pose.position.y, now_pose.position.z;
                tool_base_orientation.coeffs() << now_pose.orientation.x,
                                                now_pose.orientation.y,
                                                now_pose.orientation.z,
                                                now_pose.orientation.w;

                Eigen::Quaterniond cam_base_orientation = tool_base_orientation * cam_tool_orientatoin;
                Eigen::Vector3d cam_base_translation = tool_base_orientation.toRotationMatrix() * cam_tool_translation + tool_base_translation;

                Eigen::Quaterniond obj_base_orientation = cam_base_orientation * listener.obj_cam_orientation_;
                Eigen::Vector3d obj_base_translation = cam_base_orientation.toRotationMatrix() * listener.obj_cam_translation_ + cam_base_translation;

                Eigen::Quaterniond tcp_base_orientation = obj_base_orientation * tcp_obj_orientation;
                Eigen::Vector3d tcp_base_translation = obj_base_orientation.toRotationMatrix() * tcp_obj_translation + obj_base_translation; 

                std::cout<<"now obj position: "<< tcp_base_translation.transpose() <<std::endl;
                std::cout<<"now obj orientation: [w, x, y, z]: ["
                        << tcp_base_orientation.w() <<", "<< tcp_base_orientation.x() <<", "<<tcp_base_orientation.y()<<", "<<tcp_base_orientation.z() <<std::endl;


                std::vector<geometry_msgs::Pose> waypoints;
                geometry_msgs::Pose pose1;
                pose1.position.x = tcp_base_translation(0);
                pose1.position.y = tcp_base_translation(1);	
                pose1.position.z = tool_base_translation(2);	
                pose1.orientation.x = obj_base_orientation.x();
                pose1.orientation.y = obj_base_orientation.y();
                pose1.orientation.z = obj_base_orientation.z();
                pose1.orientation.w = obj_base_orientation.w();
                waypoints.push_back(pose1);

                geometry_msgs::Pose pose2;
                pose2.position.x = tcp_base_translation(0);
                pose2.position.y = tcp_base_translation(1);	
                pose2.position.z = tcp_base_translation(2);	
                pose2.orientation.x = obj_base_orientation.x();
                pose2.orientation.y = obj_base_orientation.y();
                pose2.orientation.z = obj_base_orientation.z();
                pose2.orientation.w = obj_base_orientation.w();
                waypoints.push_back(pose2);

                // planning
                moveit_msgs::RobotTrajectory trajectory;
                const double jump_threshold = 0.0;
                const double eef_step = 0.002;
                double fraction = 0.0;
                int maxtries = 100;   // max attempts
                int attempts = 0;     // attempts done

                while(fraction < 1.0 && attempts < maxtries)
                {
                    fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
                    attempts++;
                    if(attempts % 10 == 0)
                        ROS_INFO("Still trying after %d attempts...", attempts);
                }
                
                if(fraction == 1)
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
                grip_force = 0.7;
                grip_position = 0.0;
                geometry_msgs::Twist msgOut;
                msgOut.linear.x = grip_force;
                msgOut.linear.y = grip_position;
                pub.publish(msgOut);
                grip_flag = false;
                back_flag = true;
                sleep(2);
            }
           
        }
        if(back_flag)
        {
            std::cout<<"in the last second plan"<<std::endl;
            arm.setNamedTarget("qr_start"); //qr_state, for the best overlook view
            arm.move();
            // sleep(1);

            // arm.setNamedTarget("qr_end"); //qr_state, for the best overlook view
            // arm.move();
            back_flag = false;
            sleep(1);

            mission_mode::mode_switch goback_sig;
            goback_sig.request.mode_flag = 0;
            vnc_client.call(goback_sig);
            sleep(3);
            ROS_WARN("Finish call Goback");

            // std_srvs::Trigger launch_nav_sig;
            // launch_nav_client.call(launch_nav_sig);
            // ROS_WARN("Finish call launch nav");
        }
        if (listener.place_flag)
        {
            std::cout<<"in the last plan"<<std::endl;
            arm.setNamedTarget("ground_place"); //qr_state, for the best overlook view
            arm.move();
            done_flag = false;
            // listener.place_flag = false;
            sleep(1);
            grip_force = 0.7;
            grip_position = 0.9;
            geometry_msgs::Twist msgOut;
            msgOut.linear.x = grip_force;
            msgOut.linear.y = grip_position;
            pub.publish(msgOut);
            sleep(1);

            mission_mode::mode_switch goback_sig;
            goback_sig.request.mode_flag = 0;
            vnc_client.call(goback_sig);
            sleep(3);
            ROS_WARN("Finish call Goback");

        }
        loop_rate.sleep();
    }
    ros::waitForShutdown();

    return 0;
}