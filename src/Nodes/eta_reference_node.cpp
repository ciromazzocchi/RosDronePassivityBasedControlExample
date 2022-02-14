#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "quad_control/UavState.h"
#include "quad_control/DesiredTrajectory.h"
#include "nav_msgs/Odometry.h"
#include "../Utils/utility.hpp"
#include "../Utils/Differentiator.hpp"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

Differentiator<Eigen::Vector3d> eta_dot_filter;
Differentiator<Eigen::Vector3d> eta_dot_dot_filter;

ros::Subscriber eta_ref_sub_1, eta_ref_sub_2, eta_ref_sub_3;
ros::Publisher  eta_ref_pub;

double m, psi_d, v;
Eigen::Vector3d eta,eta_dot;

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    eta << odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z;
    
    eta_dot << odom_msg->twist.twist.angular.x,
        odom_msg->twist.twist.angular.y,
        odom_msg->twist.twist.angular.z;
}

void traj_cb(const quad_control::DesiredTrajectory::ConstPtr& traj_msg) {
    psi_d = traj_msg->position.yaw;
}

void mu_hat_cb(const geometry_msgs::Vector3Stamped::ConstPtr& mu_hat_msg) {
    Eigen::Vector3d mu_hat;
    tf::vectorMsgToEigen(mu_hat_msg->vector,mu_hat);

    double phi_d = asin( ( mu_hat(1)*cos(psi_d) - mu_hat(0)*sin(psi_d) ) /
                    ( sqrt( mu_hat.transpose() * mu_hat) ));
    
    double theta_d = atan2( mu_hat(0)*cos(psi_d) + mu_hat(1)*sin(psi_d), mu_hat(2));

    Eigen::Vector3d eta_d = Eigen::Vector3d(phi_d,theta_d,psi_d);
    Eigen::Vector3d eta_dot_d = eta_dot_filter.getDifferentiatoredValue(eta_d);
    Eigen::Vector3d eta_dot_dot_d = eta_dot_dot_filter.getDifferentiatoredValue(eta_dot_d);

    quad_control::UavState msg;
    msg.header = mu_hat_msg->header;
    tf::vectorEigenToMsg(eta_d,msg.pose);
    tf::vectorEigenToMsg(eta_dot_d,msg.twist);
    tf::vectorEigenToMsg(eta_dot_dot_d,msg.wrench);
    eta_ref_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eta_reference_node");
    ros::NodeHandle nh("~");

    double Ts   = nh.param<double>("Ts", 0.001);
    m           = nh.param<double>("mass", 0.1);
    eta         = Eigen::Vector3d::Zero();
    eta_dot     = Eigen::Vector3d::Zero();
    psi_d = 0;
    
    double kf = nh.param<double>("kf", 100);
    eta_dot_filter.DifferentiatorInit( kf, Ts );
    eta_dot_dot_filter.DifferentiatorInit( kf, Ts );

    eta_ref_sub_1 = nh.subscribe("/odometry", 1, odom_cb);
    eta_ref_sub_2 = nh.subscribe("/eta_d", 1, traj_cb);
    eta_ref_sub_3 = nh.subscribe("/mu_hat", 1, mu_hat_cb);
    eta_ref_pub   = nh.advertise<quad_control::UavState>("/eta_reference", 1);
    ros::Rate rate(1/Ts);
    
    ros::Duration(0.02).sleep();

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}