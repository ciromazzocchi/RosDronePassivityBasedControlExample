#include "ros/ros.h"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include "quad_control/State.h"
#include "geometry_msgs/Wrench.h"

#include "../Utils/utility.hpp"

ros::Subscriber est_eta_sub, est_p_sub, est_cmd_sub;
ros::Publisher est_f_pub, est_t_pub;

Eigen::Vector3d p_dot, eta, eta_dot;

Eigen::Matrix3d I,R;

double m,k0,Ts;

Eigen::Matrix<double,6,6> Cxi;

Eigen::Matrix<double,6,4> Dxi;

Eigen::Matrix<double,6,1> Gxi, F_est, Xi_dot;


void odom_eta_cb(const quad_control::State::ConstPtr& odom_msg) {
    tf::vectorMsgToEigen(odom_msg->position,eta);
    tf::vectorMsgToEigen(odom_msg->speed,eta_dot);

    Cxi << Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
        Eigen::Matrix3d::Zero(), getC(eta,eta_dot,I);

    Dxi << -getR(eta)*Eigen::Vector3d(0,0,1), Eigen::Matrix3d::Zero(),
        Eigen::Vector3d::Zero(), getQ(eta).transpose();

    Gxi << -m*9.81*Eigen::Vector3d(0,0,1), Eigen::Vector3d::Zero();
}

void odom_p_cb(const quad_control::State::ConstPtr& odom_msg) {
    tf::vectorMsgToEigen(odom_msg->speed,p_dot);
}

void cmd_cb(const geometry_msgs::Wrench::ConstPtr& cmd_msg) {
    Xi_dot << p_dot, eta_dot;
    
    Eigen::Matrix<double,4,1> u;
    u << cmd_msg->force.z, cmd_msg->torque.x, cmd_msg->torque.y, cmd_msg->torque.z;

    F_est = (1+Ts*k0)*F_est + Ts*k0*(Cxi.transpose()*Xi_dot + Dxi*u - Gxi);

    geometry_msgs::Vector3 msg;
    msg.x = F_est(0);   msg.y = F_est(1);   msg.z = F_est(2);
    est_f_pub.publish(msg);
    msg.x = F_est(3);   msg.y = F_est(4);   msg.z = F_est(5);
    est_t_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "estimator_node");
    ros::NodeHandle nh("~");

    I = Eigen::Vector3d(nh.param<double>("Ixx", 0.1), nh.param<double>("Iyy", 0.1),
            nh.param<double>("Izz", 0.1)).asDiagonal();
    
    m = nh.param<double>("mass", 0.1);
    
    k0 = nh.param<double>("k0", 1);

    Ts =  1/nh.param<double>("rate", 100);

    F_est = Eigen::Matrix<double,6,1>::Zero();

    est_eta_sub = nh.subscribe("/eta",  1, odom_eta_cb);
    est_p_sub   = nh.subscribe("/p",    1, odom_p_cb);
    est_cmd_sub = nh.subscribe("/cmd",  1, cmd_cb);

    est_f_pub = nh.advertise<geometry_msgs::Vector3>("/est/f",  1);
    est_t_pub = nh.advertise<geometry_msgs::Vector3>("/est/t",  1);

    ros::spin();

    return 0;
}