#include "ros/ros.h"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include "../Utils/Differentiator.hpp"

#include "quad_control/UavState.h"
#include "quad_control/DesiredAttritude.h"

Differentiator<Eigen::Vector3d> eta_dot_filter;
Differentiator<Eigen::Vector3d> etaDot_dot_filter;

ros::Subscriber eta_ref_sub, traj_sub;
ros::Publisher  eta_ref_pub;

double m, v, psi_d;

void traj_cb(const quad_control::UavState::ConstPtr& traj_msg) {
    psi_d = traj_msg->psi_d;
};

void mu_hat_cb(const geometry_msgs::Vector3::ConstPtr& mu_hat_msg) {
    Eigen::Vector3d mu_hat;
    tf::vectorMsgToEigen(*mu_hat_msg,mu_hat);

    mu_hat(2) = mu_hat(2) - 9.81;

    double phi_d = asin( ( mu_hat(1)*cos(psi_d) - mu_hat(0)*sin(psi_d) ) /
                    ( sqrt( mu_hat.transpose() * mu_hat) ));
    
    double theta_d = atan( ( mu_hat(0)*cos(psi_d) + mu_hat(1)*sin(psi_d) ) / mu_hat(2));

    Eigen::Vector3d eta_d = Eigen::Vector3d(phi_d,theta_d,psi_d);
    Eigen::Vector3d etaDot_d = eta_dot_filter.getDifferentiatoredValue(eta_d);
    Eigen::Vector3d etaDotDot_d = etaDot_dot_filter.getDifferentiatoredValue(etaDot_d);

    quad_control::DesiredAttritude msg;
    tf::vectorEigenToMsg(eta_d       * 180/M_PI, msg.eta);
    tf::vectorEigenToMsg(etaDot_d    * 180/M_PI, msg.eta_dot);
    tf::vectorEigenToMsg(etaDotDot_d * 180/M_PI, msg.eta_dot_dot);

    eta_ref_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eta_reference_node");
    ros::NodeHandle nh("~");

    m       = nh.param<double>("mass", 0.1);
    psi_d = 0;

    double kf_d = nh.param<double>("kf_d", 100);
    double kf_dd = nh.param<double>("kf_dd", 100);
    eta_dot_filter.DifferentiatorInit( kf_d, 0.01 );
    etaDot_dot_filter.DifferentiatorInit( kf_dd, 0.01 );

    eta_ref_sub = nh.subscribe("/sub_topic", 1, mu_hat_cb);
    traj_sub    = nh.subscribe("/traj_topic", 1, traj_cb);

    eta_ref_pub = nh.advertise<quad_control::DesiredAttritude>("/eta_ref_topic", 1);
    
    ros::spin();

    return 0;
}