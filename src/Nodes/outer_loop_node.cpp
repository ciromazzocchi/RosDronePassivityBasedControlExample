#include "ros/ros.h"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include "quad_control/UavState.h"
#include "nav_msgs/Odometry.h"

ros::Subscriber ol_odom_sub,ol_traj_sub;
ros::Publisher  ol_mu_pub;

double m;
Eigen::Vector3d p_d, p_dot_d, p_dot_dot_d;
Eigen::Matrix3d Kp, Kd;

void traj_cb(const quad_control::UavState::ConstPtr& msg) {
    tf::vectorMsgToEigen(msg->pose_d.linear,p_d);
    tf::vectorMsgToEigen(msg->twist_d.linear,p_dot_d);
    tf::vectorMsgToEigen(msg->wrench_d.linear,p_dot_dot_d);
};

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    Eigen::Vector3d p, p_dot;
    tf::pointMsgToEigen(odom_msg->pose.pose.position,p);
    tf::vectorMsgToEigen(odom_msg->twist.twist.linear,p_dot);

    Eigen::Vector3d e = p - p_d;
    Eigen::Vector3d e_dot = p_dot - p_dot_d;

    Eigen::Vector3d force_est = Eigen::Vector3d::Zero();
    
    Eigen::Vector3d mu = p_dot_dot_d - (1/m)*(Kp*e + Kd*e_dot)  - (1/m)*force_est;

    if(mu[2] >  9) mu[2] =  9;
    if(mu[2] < -9) mu[2] = -9;

    geometry_msgs::Vector3 msg;
    tf::vectorEigenToMsg(mu,msg);
    ol_mu_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "outer_loop_node");
    ros::NodeHandle nh("~");

    Kp  = nh.param<double>("Kp", 1) * Eigen::Matrix3d::Identity();
    Kd  = nh.param<double>("Kd", 1) * Eigen::Matrix3d::Identity();
    m   = nh.param<double>("mass", 0.1);

    p_d = p_dot_d = p_dot_dot_d = Eigen::Vector3d::Zero();

    ol_odom_sub = nh.subscribe("/odom_topic", 1, odom_cb);
    ol_traj_sub = nh.subscribe("/traj_topic", 1, traj_cb);
    ol_mu_pub   = nh.advertise<geometry_msgs::Vector3>("/mu_hat", 1);

    ros::spin();

    return 0;
}