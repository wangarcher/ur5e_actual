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



class Listener
{
public:
    float tran_pose_x;
    float tran_pose_y;
    float tran_pose_z;
    float tran_quat_x;
    float tran_quat_y;
    float tran_quat_z;
    float tran_quat_w;
    int move_flag = 0;
    int view_flag = 0;
public:
    void flag_callback(const std_msgs::Int8& flag);
    void obj_callback(const nav_msgs::Odometry& odom);
};

void Listener::flag_callback(const std_msgs::Int8& flag)
{
    if(flag.data == 1)
    {
        ROS_INFO("Received a flag message!");
        move_flag = 1;
    }
}

void Listener::obj_callback(const nav_msgs::Odometry& odom)
{
    if(odom.header.frame_id == "ok")
    {
        tran_pose_x = odom.pose.pose.position.x;
        tran_pose_y = odom.pose.pose.position.y;
        tran_pose_z = odom.pose.pose.position.z;
        tran_quat_x = odom.pose.pose.orientation.x;
        tran_quat_y = odom.pose.pose.orientation.y;
        tran_quat_z = odom.pose.pose.orientation.z;
        tran_quat_w = odom.pose.pose.orientation.w; 
        view_flag = 1;
    }
}


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "project_plan");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(2);
	spinner.start();

    bool qr_flag = true;
    bool grip_flag = true;

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
    ros::Subscriber sub1 = n.subscribe("/move_flag", 1000, &Listener::flag_callback, &listener);
    ros::Subscriber sub2 = n.subscribe("/obj_pose", 1000, &Listener::obj_callback, &listener);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/grip_control", 1);

    while(qr_flag || grip_flag)
    {
    if(listener.move_flag == 1 && qr_flag)
    //if(qr_flag)
    {
        std::cout<<"in the frist plan"<<std::endl;
        qr_flag = false;
        arm.setNamedTarget("qr_start"); //qr_state, for the best overlook view
        arm.move();
    }

    if(listener.view_flag == 1 && grip_flag)
    //if(grip_flag)
    {   
        grip_flag = false;
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
        Eigen::Quaterniond q1(now_pose.orientation.w, now_pose.orientation.x, now_pose.orientation.y, now_pose.orientation.z);
        Eigen::Quaterniond q2(listener.tran_quat_w, listener.tran_quat_x, listener.tran_quat_y, listener.tran_quat_z);
        Eigen::Quaterniond q;
        q = q1 * q2;
        q.normalized();

        std::vector<geometry_msgs::Pose> waypoints;
        geometry_msgs::Pose pose1;
        pose1.position.x = now_pose.position.x + listener.tran_pose_x;
        pose1.position.y = now_pose.position.y + listener.tran_pose_y;	
        pose1.position.z = now_pose.position.z;
        pose1.orientation.x = q.x();
        pose1.orientation.y = q.y();
        pose1.orientation.z = q.z();
        pose1.orientation.w = q.w();
        waypoints.push_back(pose1);

        geometry_msgs::Pose pose2;
        pose2.position.x = now_pose.position.x;
        pose2.position.y = now_pose.position.y;	
        pose2.position.z = now_pose.position.z + listener.tran_pose_z;
        pose2.orientation.x = q.x();
        pose2.orientation.y = q.y();
        pose2.orientation.z = q.z();
        pose2.orientation.w = q.w();
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
    

        grip_force = 1.0;
        grip_position = (400-340)*1.0/(1700-340);
        if(grip_position > 1)
            grip_position = 1.0;
        else if (grip_position < 0)
            grip_position = 0.0;
        geometry_msgs::Twist msgOut;
        msgOut.linear.x = grip_force;
        msgOut.linear.y = grip_position;
        pub.publish(msgOut);
    }
    loop_rate.sleep();
    }
    ros::waitForShutdown();

    return 0;
}