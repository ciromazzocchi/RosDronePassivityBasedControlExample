#include "ros/ros.h"

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include "../Utils/utility.hpp"

#include "quad_control/State.h"

ros::Subscriber il_sub, il_odom_sub, il_mu_sub;
ros::Publisher  il_cmd_pub;

double m, v;
Eigen::Matrix3d Kp, Kd, I, M, C, Qt_inv;
Eigen::Vector3d eta,eta_dot, mu_hat;

void mu_sub_cb(const geometry_msgs::Vector3::ConstPtr& msg) {
    mu_hat << msg->x, msg->y, msg->z;

    mu_hat(2)= mu_hat(2) - 9.81;
}

void odom_sub_cb(const quad_control::State::ConstPtr& msg) {
    tf::vectorMsgToEigen(msg->position, eta);
    tf::vectorMsgToEigen(msg->speed, eta_dot);

    Qt_inv = getQ(eta).transpose().inverse();
    M = getM(eta,I);
    C = getC(eta,eta_dot, I);
}

void cb(const quad_control::State::ConstPtr& traj_msg) {
    double uT = m*sqrt( mu_hat.transpose() * mu_hat);

    Eigen::Vector3d eta_d,eta_dot_d, eta_dot_dot_d;

    tf::vectorMsgToEigen(traj_msg->position,        eta_d);
    tf::vectorMsgToEigen(traj_msg->speed,           eta_dot_d);
    tf::vectorMsgToEigen(traj_msg->acceleration,    eta_dot_dot_d);

    Eigen::Vector3d e       = eta       - eta_d;
    Eigen::Vector3d e_dot   = eta_dot   - eta_dot_d;

    Eigen::Vector3d eta_dot_r       = eta_dot_d     - v*e;
    Eigen::Vector3d eta_dot_dot_r   = eta_dot_dot_d - v*e_dot;
    Eigen::Vector3d v_eta           = e_dot         + v*e;

    Eigen::Vector3d tau_est = Eigen::Vector3d::Zero();
    
    Eigen::Matrix<double,6,1> wrench;
    wrench << Eigen::Vector3d(0,0,uT),
            Qt_inv * (M*eta_dot_dot_r + C*eta_dot_r - tau_est - Kd*v_eta - Kp*e);
    geometry_msgs::Wrench msg;
    tf::wrenchEigenToMsg(wrench, msg);
    il_cmd_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inner_loop_node");

    ros::NodeHandle nh("~");

    Kp = Eigen::Vector3d(nh.param<double>("Kp_x", 1), nh.param<double>("Kp_y", 1),
            nh.param<double>("Kp_z", 1)).asDiagonal();
    Kd = Eigen::Vector3d(nh.param<double>("Kd_x", 1), nh.param<double>("Kd_y", 1),
            nh.param<double>("Kd_z", 1)).asDiagonal();
    m   = nh.param<double>("mass", 0.1);
    I = Eigen::Vector3d(nh.param<double>("Ixx", 0.1), nh.param<double>("Iyy", 0.1),
            nh.param<double>("Izz", 0.1)).asDiagonal();

    v = nh.param<double>("v", 1);

    eta = eta_dot = mu_hat = Eigen::Vector3d::Zero();

    M = C = Qt_inv = Eigen::Matrix3d::Zero();

    il_sub      = nh.subscribe("/eta_ref_topic", 1, cb);
    il_odom_sub = nh.subscribe("/odom_topic", 1, odom_sub_cb);
    il_mu_sub   = nh.subscribe("/mu_topic", 1, mu_sub_cb);
    il_cmd_pub  = nh.advertise<geometry_msgs::Wrench>("/pub_topic", 1);
    
    ros::spin();

    return 0;
}