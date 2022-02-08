#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "quad_control/UavState.h"
#include "quad_control/DesiredTrajectory.h"
#include "../Utils/utility.hpp"
#include "../Utils/Differentiator.hpp"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

Differentiator<Eigen::Vector3d> eta_dot_filter;
Differentiator<Eigen::Vector3d> eta_dot_dot_filter;

geometry_msgs::Vector3 msg;
ros::Subscriber outerloop_sub_1, outerloop_sub_2, outerloop_sub_3, outerloop_sub_4;
ros::Publisher  outerloop_pub_1;
ros::Publisher  outerloop_pub_2;
ros::Publisher  outerloop_pub_3;
ros::Publisher  outerloop_pub_4;

double m, psi_d, v;
Eigen::Vector3d eta,eta_dot, eta_dot_dot, tau_est;
Eigen::Matrix3d Kp, Kd, I;

void outerloop_callback_1(const quad_control::UavState::ConstPtr& odom_msg) {
    eta <<  odom_msg ->pose.angular.x,
            odom_msg ->pose.angular.y,
            odom_msg ->pose.angular.z;
    
    eta_dot <<  odom_msg ->twist.angular.x,
                odom_msg ->twist.angular.y,
                odom_msg ->twist.angular.z;

    eta_dot_dot <<  odom_msg ->wrench.angular.x,
                    odom_msg ->wrench.angular.y,
                    odom_msg ->wrench.angular.z;
}

void outerloop_callback_2(const geometry_msgs::Vector3::ConstPtr& tau_msg) {
    tau_est << tau_msg->x, tau_msg->y, tau_msg->z;
}

void outerloop_callback_3(const quad_control::DesiredTrajectory::ConstPtr& traj_msg) {
    psi_d = traj_msg->position.yaw;
}

void outerloop_callback_4(const geometry_msgs::Vector3::ConstPtr& mu_hat_msg) {
    Eigen::Vector3d mu_hat = Eigen::Vector3d( mu_hat_msg->x, mu_hat_msg->y, mu_hat_msg->z);

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
    outerloop_pub_1.publish(msg);
    geometry_msgs::Vector3 vec_msg;
    tf::vectorEigenToMsg(e, vec_msg);
    outerloop_pub_2.publish(vec_msg);
    tf::vectorEigenToMsg(e_dot, vec_msg);
    outerloop_pub_3.publish(vec_msg);
    tf::vectorEigenToMsg(e_dot_dot, vec_msg);
    outerloop_pub_4.publish(vec_msg);
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
    
    psi_d = double(0);
    
    v = nh.param<double>("v", 10);

    double kf = nh.param<double>("kf", 100);
    eta_dot_filter.DifferentiatorInit( kf, Ts );
    eta_dot_dot_filter.DifferentiatorInit( kf, Ts );

    outerloop_sub_1 = nh.subscribe("/odometry", 1, outerloop_callback_1);
    outerloop_sub_2 = nh.subscribe("/wrenchEstimator", 1, outerloop_callback_2);
    outerloop_sub_3 = nh.subscribe("/eta_d", 1, outerloop_callback_3);
    outerloop_sub_4 = nh.subscribe("/mu_hat", 1, outerloop_callback_4);
    outerloop_pub_1   = nh.advertise<geometry_msgs::Wrench>("/wrenchNED", 1);
    outerloop_pub_2   = nh.advertise<geometry_msgs::Vector3>("/error_pose", 1);
    outerloop_pub_3   = nh.advertise<geometry_msgs::Vector3>("/error_twist", 1);
    outerloop_pub_4   = nh.advertise<geometry_msgs::Vector3>("/error_wrench", 1);
    
    ros::Duration(2).sleep();

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}