// a simple planning demo, initialized in 210813
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <vector>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>



class RecoBuffer
{
    public:
        int reco_flag; // outwards flag, when the ee reached the desired pre-picking place
        
    public:
        void reco_trigger_callback(const std_msgs::Int8 & trigger);
};


void RecoBuffer::reco_trigger_callback(const std_msgs::Int8 & trigger)
{
    if (trigger.data == 1)
    {
        reco_flag = 1;
    }
    if (trigger.data == 2)
    {
        reco_flag = 2;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "reco_buffer_pipeline");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(7);
	spinner.start();

    // some flag were set here, so as to make sure the sequence flow running in a proper way.
    // initialization
    RecoBuffer reco_buffer;

    reco_buffer.reco_flag = 0;
    bool flag = true;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    Eigen::Vector3d cam_obj_translation_;
    Eigen::Vector4d cam_obj_quaternion_;
    Eigen::Matrix4d M_;
    Eigen::Vector3d T_;

    ros::Rate loop_rate(10);

    ros::Subscriber sub_trigger = n.subscribe("/reco_trigger", 1, &RecoBuffer::reco_trigger_callback, &reco_buffer);
    ros::Publisher  pub_pose = n.advertise<geometry_msgs::PoseStamped>("/object_visual_odometry", 1);

    int count_down = 20;
    M_.setZero();
    T_.setZero(); 
    while(n.ok())
    {  
        if(reco_buffer.reco_flag != 0)
        {   
            while(count_down > 0)
            {
                try
                {
                    listener.lookupTransform("/camera_base", "/marker_1", ros::Time(0), transform);
                    count_down--;
                }
                catch (tf::TransformException ex)
                {
                    ROS_ERROR("%s",ex.what());
                    continue;
                }
                geometry_msgs::Pose pose_midterm;
                tf::poseTFToMsg(transform, pose_midterm);
                cam_obj_translation_ << pose_midterm.position.x,
                                        pose_midterm.position.y,
                                        pose_midterm.position.z;
                cam_obj_quaternion_ << pose_midterm.orientation.x,
                                       pose_midterm.orientation.y,
                                       pose_midterm.orientation.z,
                                       pose_midterm.orientation.w;
                // std::cout << "cam_obj_translation_: "<< cam_obj_translation_.transpose() << std::endl;
                // std::cout << "cam_obj_quaternion_: " << cam_obj_quaternion_.transpose() << std::endl; 

                M_ = M_ + cam_obj_quaternion_ * cam_obj_quaternion_.transpose();
                T_ = T_ + cam_obj_translation_;
                sleep(0.05);
            }
            M_ = M_ / 20;
            T_ = T_ / 20;

            // get the average value of the quaternion
            Eigen::EigenSolver<Eigen::Matrix4d> es(M_);
            Eigen::Matrix4d evals = es.pseudoEigenvalueMatrix();//get the eigen vectors, 4*4 matrix
            Eigen::Matrix4d evecs = es.pseudoEigenvectors();//get the eigen values
            // std::cout << "evals: " << evals.transpose() << std::endl; 
            // std::cout << "evecs: " << evecs.transpose() << std::endl; 
            int row_index, col_index;
            evals.maxCoeff(&row_index, &col_index);
            Eigen::Vector4d q;
            q << evecs.real()(0, col_index), 
                 evecs.real()(1, col_index), 
                 evecs.real()(2, col_index), 
                 evecs.real()(3, col_index);
            std::cout << "T_: " << T_.transpose() << std::endl; 
            std::cout << "q: " << q.transpose() << std::endl; 
            geometry_msgs::PoseStamped output_pose;
            output_pose.pose.position.x = T_(0);
            output_pose.pose.position.y = T_(1);
            output_pose.pose.position.z = T_(2);
            output_pose.pose.orientation.x = q(0);
            output_pose.pose.orientation.y = q(1);
            output_pose.pose.orientation.z = q(2);
            output_pose.pose.orientation.w = q(3);
            if (reco_buffer.reco_flag == 1)
            {
                output_pose.header.frame_id = "1st";
            }
            if (reco_buffer.reco_flag == 2)
            {
                output_pose.header.frame_id = "2nd";
            }
            for (int i = 5; i > 0; i --)
            {
                pub_pose.publish(output_pose);
            }
            reco_buffer.reco_flag = 0;
        }
        M_.setZero();
        T_.setZero(); 
        count_down = 20;
    }
    std::cout<<"Info: mission accomplished!!!"<<std::endl;
    ros::waitForShutdown();

    return 0;
}