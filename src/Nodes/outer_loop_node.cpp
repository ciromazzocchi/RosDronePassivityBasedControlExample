#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "nav_msgs/Odometry.h"
#include "quad_control/UavState.h"
#include "quad_control/DesiredTrajectory.h"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

ros::Subscriber ol_odom_sub, ol_wrench_est_sub, ol_traj_sub;
ros::Publisher  ol_mu_pub, ol_err_pose_pub, ol_err_twist_pub;

double m;
Eigen::Vector3d force_est, p_d, p_dot_d, p_dot_dot_d;
Eigen::Matrix3d Kp, Kd;

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    Eigen::Vector3d p, p_dot, p_dot_dot;
    p << odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z;
    
    p_dot << odom_msg->twist.twist.linear.x,
            odom_msg->twist.twist.linear.y,
            odom_msg->twist.twist.linear.z;

    Eigen::Vector3d e           = p         - p_d;
    Eigen::Vector3d e_dot       = p_dot     - p_dot_d;
    Eigen::Vector3d mu = p_dot_dot_d - (1/m)*(Kp*e + Kd*e_dot)  - (1/m)*force_est;

    if(mu[2] >  9) mu[2] =  9;
    if(mu[2] < -9) mu[2] = -9;

    geometry_msgs::Vector3Stamped msg;
    msg.header = odom_msg->header;
    tf::vectorEigenToMsg(mu, msg.vector);
    ol_mu_pub.publish(msg);
    tf::vectorEigenToMsg(e, msg.vector);
    ol_err_pose_pub.publish(msg);
    tf::vectorEigenToMsg(e_dot, msg.vector);
    ol_err_twist_pub.publish(msg);
}

void wrench_est_cb(const geometry_msgs::Wrench::ConstPtr& force_msg) {
    force_est.x() = force_msg->force.x;
    force_est.y() = force_msg->force.y;
    force_est.z() = force_msg->force.z;
}

void traj_cb(const quad_control::DesiredTrajectory::ConstPtr& traj_msg) {
    p_d.x() = traj_msg->position.x;
    p_d.y() = traj_msg->position.y;
    p_d.z() = traj_msg->position.z;

    p_dot_d.x() = traj_msg->twist.x;
    p_dot_d.y() = traj_msg->twist.y;
    p_dot_d.z() = traj_msg->twist.z;
    
    p_dot_dot_d.x() = traj_msg->wrench.x;
    p_dot_dot_d.y() = traj_msg->wrench.y;
    p_dot_dot_d.z() = traj_msg->wrench.z;
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
    
    p_d         = Eigen::Vector3d(0,0,-0.06);
    p_dot_d     = Eigen::Vector3d(0,0,0);
    p_dot_dot_d = Eigen::Vector3d(0,0,0);

    ol_odom_sub         = nh.subscribe("/odometryNED", 1, odom_cb);
    ol_wrench_est_sub   = nh.subscribe("/wrenchEstimator", 1, wrench_est_cb);
    ol_traj_sub         = nh.subscribe("/trajectory", 1, traj_cb);
    ol_mu_pub           = nh.advertise<geometry_msgs::Vector3Stamped>("/mu_hat", 1);
    ol_err_pose_pub     = nh.advertise<geometry_msgs::Vector3Stamped>("/error_pose", 1);
    ol_err_twist_pub    = nh.advertise<geometry_msgs::Vector3Stamped>("/error_twist", 1);
    
    ros::Duration(0.01).sleep();

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}