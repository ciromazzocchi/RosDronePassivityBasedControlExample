#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "quad_control/UavState.h"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

geometry_msgs::Vector3 msg;
ros::Subscriber outerloop_sub_1;
ros::Subscriber outerloop_sub_2;
ros::Publisher  outerloop_pub_1;
ros::Publisher  outerloop_pub_2;
ros::Publisher  outerloop_pub_3;
ros::Publisher  outerloop_pub_4;

double m;
Eigen::Vector3d force_est, p_d, p_dot_d, p_dot_dot_d;
Eigen::Matrix3d Kp, Kd;

void outerloop_callback_1(const quad_control::UavState::ConstPtr& odom_msg) {
    Eigen::Vector3d mu_hat;
    Eigen::Vector3d p           = Eigen::Vector3d(  odom_msg->pose.linear.x,
                                                    odom_msg->pose.linear.y,
                                                    odom_msg->pose.linear.z);

    Eigen::Vector3d p_dot       = Eigen::Vector3d(  odom_msg->twist.linear.x,
                                                    odom_msg->twist.linear.y,
                                                    odom_msg->twist.linear.z);

    Eigen::Vector3d p_dot_dot   = Eigen::Vector3d(  odom_msg->wrench.linear.x,
                                                    odom_msg->wrench.linear.y,
                                                    odom_msg->wrench.linear.z);

    Eigen::Vector3d e           = p         - p_d;
    Eigen::Vector3d e_dot       = p_dot     - p_dot_d;
    Eigen::Vector3d e_dot_dot   = p_dot_dot - p_dot_dot_d;
    Eigen::Vector3d mu = p_dot_dot_d - (1/m)*(Kp*e + Kd*e_dot)  - (1/m)*force_est;

    if(mu[2] >  0.90) mu[2] =  0.90;
    if(mu[2] < -0.90) mu[2] = -0.90;

    tf::vectorEigenToMsg(mu, msg);
    outerloop_pub_1.publish(msg);
    tf::vectorEigenToMsg(e, msg);
    outerloop_pub_2.publish(msg);
    tf::vectorEigenToMsg(e_dot, msg);
    outerloop_pub_3.publish(msg);
    tf::vectorEigenToMsg(e_dot_dot, msg);
    outerloop_pub_4.publish(msg);
}

void outerloop_callback_2(const geometry_msgs::Wrench::ConstPtr& force_msg) {
    force_est.x() = force_msg->force.x;
    force_est.y() = force_msg->force.y;
    force_est.z() = force_msg->force.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "outer_loop_node");
    ros::NodeHandle nh("~");

    double Ts = nh.param<double>("Ts", 0.001);
    ros::Rate rate(1/Ts);

    Kp  = nh.param<double>("Kp", 1) * Eigen::Matrix3d::Identity();
    Kd  = nh.param<double>("Kd", 1) * Eigen::Matrix3d::Identity();
    m   = nh.param<double>("mass", 0.1);
    
    p_d         = Eigen::Vector3d(0,0,-1);
    p_dot_d     = Eigen::Vector3d(0,0,0);
    p_dot_dot_d = Eigen::Vector3d(0,0,0);

    outerloop_sub_1 = nh.subscribe("/odometryNED", 1, outerloop_callback_1);
    outerloop_sub_2 = nh.subscribe("/wrenchEstimator", 1, outerloop_callback_2);
    outerloop_pub_1   = nh.advertise<geometry_msgs::Vector3>("/mu_hat", 1);
    outerloop_pub_2   = nh.advertise<geometry_msgs::Vector3>("/error_pose", 1);
    outerloop_pub_3   = nh.advertise<geometry_msgs::Vector3>("/error_twist", 1);
    outerloop_pub_4   = nh.advertise<geometry_msgs::Vector3>("/error_wrench", 1);
    
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}