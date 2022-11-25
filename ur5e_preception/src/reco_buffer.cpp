// a simple planning demo, initialized in 210813
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



class RecoBuffer
{
    public:
        Eigen::Vector3d cam_obj_translation_;
        Eigen::Vector4d cam_obj_quaternion_;
        Eigen::Matrix4d M_;
        Eigen::Vector3d T_;
        int reco_flag; // outwards flag, when the ee reached the desired pre-picking place
        int times_received;
        geometry_msg
        
    public:
        void obj_visual_callback(const geometry_msgs::PoseStamped & odom);
};

void RecoBuffer::obj_visual_callback(const geometry_msgs::PoseStamped & odom)
{
    if (reco_flag == 1 || reco_flag == 2)
    {
        cam_obj_translation_ << odom.pose.position.x,
                                odom.pose.position.y,
                                odom.pose.position.z;
        cam_obj_quaternion_ <<  odom.pose.orientation.x,
                                odom.pose.orientation.y,
                                odom.pose.orientation.z,
                                odom.pose.orientation.w;
                                        
        M_ = M_ + cam_obj_quaternion_ * cam_obj_quaternion_.transpose();
        T_ = T_ + cam_obj_translation_;
        times_received++;
    }
}

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
    reco_buffer.times_received = 0;
    reco_buffer.M_.setZero();
    reco_buffer.reco_flag = 0;


    ros::Rate loop_rate(10);

    ros::Subscriber sub_pose = n.subscribe("/leava_alone", 1000, &RecoBuffer::obj_visual_callback, &reco_buffer);
    ros::Subscriber sub_trigger = n.subscribe("/reco_trigger", 1, &RecoBuffer::reco_trigger_callback, &reco_buffer);
    ros::Publisher  pub_pose = n.advertise<geometry_msgs::PoseStamped>("/object_visual_odometry", 1);

    while(n.ok())
    {  
        if(reco_flag != 0)
        {
            // hold 5 seconds
            sleep(5);        
            reco_buffer.M_ = reco_buffer.M_ / reco_buffer.times_received;
            Eigen::EigenSolver<Matrix<double, 4, 4>> es(reco_buffer.M_);
            Eigen::Matrix4cd evecs = es.eigenvectors();//get the eigen vectors, 4*4 matrix
            Eigen::Matrix4cd evals = es.eigenvalues();//get the eigen values
            Eigen::Matrix4cd evalsReal;//注意这里定义的MatrixXd里没有c
            evalsReal=evals.real()
            Matrix4f::Index evalsMax;
            evalsReal.rowwise().sum().maxCoeff(&evalsMax);//得到最大特征值的位置
            Eigen::Vector4d q;
            q << evecs.real()(0, evalsMax), 
                evecs.real()(1, evalsMax), 
                evecs.real()(2, evalsMax), 
                evecs.real()(3, evalsMax);
            reco_buffer.T_ = reco_buffer.T_ / reco_buffer.times_received;
            
            geometry_msgs::output_pose;
            output_pose.pose.position.x = reco_buffer.T_(0);
            output_pose.pose.position.y = reco_buffer.T_(1);
            output_pose.pose.position.z = reco_buffer.T_(2);
            output_pose.pose.orientation.x = q(0);
            output_pose.pose.orientation.y = q(1);
            output_pose.pose.orientation.z = q(2);
            output_pose.pose.orientation.w = q(3);
            if (reco_flag == 1)
            {
                output_pose.header.frame_id = "1st";
            }
            if (reco_flag == 2)
            {
                output_pose.header.frame_id = "2nd";
            }
            pub_pose.publish(output_pose);
            reco_flag = 0;
        }
    }
    std::cout<<"Info: mission accomplished!!!"<<std::endl;
    ros::waitForShutdown();

    return 0;
}