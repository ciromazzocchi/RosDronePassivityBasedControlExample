#include "ros/ros.h"
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
Differentiator<Eigen::Vector3d> etaDot_dot_filter;

ros::Subscriber eta_ref_sub;
ros::Publisher  eta_ref_pub;

double m, v;

void mu_hat_cb(const quad_control::UavState::ConstPtr& mu_hat_msg) {
    Eigen::Vector3d mu_hat;
    tf::vectorMsgToEigen(mu_hat_msg->mu_hat,mu_hat);

    mu_hat(2) = mu_hat(2) - 9.81;
    
    double psi_d = mu_hat_msg->psi_d;

    double phi_d = asin( ( mu_hat(1)*cos(psi_d) - mu_hat(0)*sin(psi_d) ) /
                    ( sqrt( mu_hat.transpose() * mu_hat) ));
    
    double theta_d = atan( ( mu_hat(0)*cos(psi_d) + mu_hat(1)*sin(psi_d) ) / mu_hat(2));

    Eigen::Vector3d eta_d = Eigen::Vector3d(phi_d,theta_d,psi_d);
    Eigen::Vector3d etaDot_d = eta_dot_filter.getDifferentiatoredValue(eta_d);
    Eigen::Vector3d etaDotDot_d = etaDot_dot_filter.getDifferentiatoredValue(etaDot_d);

    quad_control::UavState msg;
    msg = *mu_hat_msg;
    tf::vectorEigenToMsg(eta_d,msg.pose_d.angular);
    tf::vectorEigenToMsg(etaDot_d,msg.twist_d.angular);
    tf::vectorEigenToMsg(etaDotDot_d,msg.wrench_d.angular);

    Eigen::Vector3d e,eDot;
    e = Eigen::Vector3d(mu_hat_msg->pose.angular.x, mu_hat_msg->pose.angular.y,
        mu_hat_msg->pose.angular.z) - eta_d;
    
    eDot = Eigen::Vector3d(mu_hat_msg->twist.angular.x, mu_hat_msg->twist.angular.y,
        mu_hat_msg->twist.angular.z) - etaDot_d;
    
    tf::vectorEigenToMsg(e,msg.error_pose.angular);
    tf::vectorEigenToMsg(eDot,msg.error_twist.angular);

    eta_ref_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eta_reference_node");
    ros::NodeHandle nh("~");

    m       = nh.param<double>("mass", 0.1);
    
    double kf_d = nh.param<double>("kf_d", 100);
    double kf_dd = nh.param<double>("kf_dd", 100);
    eta_dot_filter.DifferentiatorInit( kf_d, 0.001 );
    etaDot_dot_filter.DifferentiatorInit( kf_dd, 0.001 );

    eta_ref_sub = nh.subscribe("/mu_hat", 1, mu_hat_cb);
    eta_ref_pub   = nh.advertise<quad_control::UavState>("/eta_ref", 1);
    ros::Rate rate(1000);
    
    ros::Duration(0.02).sleep();

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}