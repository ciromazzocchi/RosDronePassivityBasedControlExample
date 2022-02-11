#include "ros/ros.h"

#include "../Utils/Differentiator.hpp"
#include "../Utils/utility.hpp"
#include "quad_control/UavState.h"
#include "quad_control/DesiredTrajectory.h"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

Differentiator<Eigen::Vector3d> eta_dot_filter;
Differentiator<Eigen::Vector3d> eta_dot_dot_filter;

ros::Subscriber il_odom_sub, il_wrench_est_sub, il_eta_d_sub, il_mu_hat_sub;
ros::Publisher  il_cmd_pub;
ros::Publisher  il_e_eta_pub;
ros::Publisher  il_e_eta_d_pub;
ros::Publisher  il_e_eta_dd_pub;

double m, psi_d, v;
Eigen::Vector3d eta,eta_dot, eta_dot_dot, tau_est;
Eigen::Matrix3d Kp, Kd, I;

void il_cb_1(const quad_control::UavState::ConstPtr& odom_msg) {
    eta <<  odom_msg->pose.x, odom_msg->pose.y, odom_msg->pose.z;
    
    eta_dot <<  odom_msg->twist.x, odom_msg->twist.y, odom_msg->twist.z;

    eta_dot_dot <<  odom_msg->wrench.x, odom_msg->wrench.y, odom_msg->wrench.z;
}

void il_cb_2(const geometry_msgs::Vector3::ConstPtr& tau_msg) {
    tau_est << tau_msg->x, tau_msg->y, tau_msg->z;
}

void il_cb_3(const quad_control::DesiredTrajectory::ConstPtr& traj_msg) {
    psi_d = traj_msg->position.yaw;
}

void il_cb_4(const geometry_msgs::Vector3Stamped::ConstPtr& mu_hat_msg) {
    Eigen::Vector3d mu_hat = Eigen::Vector3d( mu_hat_msg->vector.x,
                                    mu_hat_msg->vector.y, mu_hat_msg->vector.z);

    double uT = m*sqrt( mu_hat.transpose() * mu_hat);

    double phi_d = asin( ( mu_hat(1)*cos(psi_d) - mu_hat(0)*sin(psi_d) ) /
                    ( sqrt( mu_hat.transpose() * mu_hat) ));
    
    double theta_d = atan2( mu_hat(0)*cos(psi_d) + mu_hat(1)*sin(psi_d), mu_hat(2));

    Eigen::Vector3d eta_d = Eigen::Vector3d(phi_d,theta_d,psi_d);
    Eigen::Vector3d eta_dot_d = eta_dot_filter.getDifferentiatoredValue(eta_d);
    Eigen::Vector3d eta_dot_dot_d = eta_dot_filter.getDifferentiatoredValue(eta_dot_d);

    Eigen::Vector3d e         = eta         - eta_d;
    Eigen::Vector3d e_dot     = eta_dot     - eta_dot_d;
    Eigen::Vector3d e_dot_dot = eta_dot_dot - eta_dot_dot_d;

    Eigen::Vector3d eta_r_dot     = eta_dot_d - v*e;
    Eigen::Vector3d eta_r_dot_dot = eta_dot_dot_d - v*e_dot;
    Eigen::Vector3d v_eta = e_dot + v*eta;

    Eigen::Matrix3d Q = getQ(eta);
    Eigen::Vector3d tau_b = Q.transpose().inverse() * (
        M(eta,I)*eta_r_dot_dot + C(eta,eta_dot, I)*eta_r_dot - tau_est - Kd*v_eta - Kp*e
    );

    geometry_msgs::Wrench msg;
    tf::vectorEigenToMsg(Eigen::Vector3d(0,0,uT), msg.force);
    tf::vectorEigenToMsg(tau_b, msg.torque);
    il_cmd_pub.publish(msg);
    geometry_msgs::Vector3Stamped vec_msg;
    vec_msg.header = mu_hat_msg->header;
    tf::vectorEigenToMsg(e, vec_msg.vector);
    il_e_eta_pub.publish(vec_msg.vector);
    tf::vectorEigenToMsg(e_dot, vec_msg.vector);
    il_e_eta_d_pub.publish(vec_msg.vector);
    tf::vectorEigenToMsg(e_dot_dot, vec_msg.vector);
    il_e_eta_dd_pub.publish(vec_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inner_loop_node");
    ros::NodeHandle nh("~");

    double Ts = nh.param<double>("Ts", 0.001);
    ros::Rate rate(1/Ts);

    Kp  = nh.param<double>("Kp", 1) * Eigen::Matrix3d::Identity();
    Kd  = nh.param<double>("Kd", 1) * Eigen::Matrix3d::Identity();
    m   = nh.param<double>("mass", 0.1);
    I = Eigen::Vector3d(nh.param<double>("Ixx", 0.1),
                        nh.param<double>("Iyy", 0.1),
                        nh.param<double>("Izz", 0.1)).asDiagonal();
    tau_est     = Eigen::Vector3d::Zero();
    psi_d = 0;
    
    v = nh.param<double>("v", 10);

    double kf = nh.param<double>("kf", 100);
    eta_dot_filter.DifferentiatorInit( kf, Ts );
    eta_dot_dot_filter.DifferentiatorInit( kf, Ts );

    il_odom_sub         = nh.subscribe("/odometry", 1, il_cb_1);
    il_wrench_est_sub   = nh.subscribe("/wrenchEstimator", 1, il_cb_2);
    il_eta_d_sub        = nh.subscribe("/eta_d", 1, il_cb_3);
    il_mu_hat_sub       = nh.subscribe("/mu_hat", 1, il_cb_4);
    il_cmd_pub          = nh.advertise<geometry_msgs::Wrench>("/wrenchNED", 1);
    il_e_eta_pub        = nh.advertise<geometry_msgs::Vector3Stamped>("/error_pose", 1);
    il_e_eta_d_pub      = nh.advertise<geometry_msgs::Vector3Stamped>("/error_twist", 1);
    il_e_eta_dd_pub     = nh.advertise<geometry_msgs::Vector3Stamped>("/error_wrench", 1);
    
    ros::Duration(0.03).sleep();

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}