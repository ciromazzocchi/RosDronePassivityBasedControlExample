#include "ros/ros.h"

#include "../Utils/utility.hpp"
#include "quad_control/UavState.h"

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

ros::Subscriber il_sub;
ros::Publisher  il_pub;

double m, v;
Eigen::Matrix3d Kp, Kd, I;

void odom_cb(const quad_control::UavState::ConstPtr& odom_msg) {
    
    Eigen::Vector3d mu_hat;
    tf::vectorMsgToEigen(odom_msg->mu_hat,mu_hat);
    mu_hat(2)= mu_hat(2) - 9.81;

    double uT = m*sqrt( mu_hat.transpose() * mu_hat);

    Eigen::Vector3d eta, etaDot;

    tf::vectorMsgToEigen(odom_msg->pose.angular,eta);
    tf::vectorMsgToEigen(odom_msg->twist.angular,etaDot);

    Eigen::Matrix3d Q = getQ(eta);
    Eigen::Matrix3d Qt = Q.transpose();
    Eigen::Matrix3d Qt_inv = Qt.inverse();

    Eigen::Matrix3d M = getM(eta,I);
    Eigen::Matrix3d C = getC(eta,etaDot, I);

    Eigen::Vector3d e, eDot, etaDot_d, etaDotDot_d;
    tf::vectorMsgToEigen(odom_msg->error_pose.angular,  e           );
    tf::vectorMsgToEigen(odom_msg->error_twist.angular, eDot        );
    tf::vectorMsgToEigen(odom_msg->twist_d.angular,     etaDot_d    );
    tf::vectorMsgToEigen(odom_msg->wrench_d.angular,    etaDotDot_d );
    
    Eigen::Vector3d etaDot_r    = etaDot_d    - v*e;
    Eigen::Vector3d etaDotDot_r = etaDotDot_d - v*eDot;
    Eigen::Vector3d v_eta       = eDot        + v*e;

    Eigen::Vector3d tau_est = Eigen::Vector3d::Zero();
    
    Eigen::Matrix<double,6,1> wrench;
    wrench << Eigen::Vector3d(0,0,uT),
            Qt_inv * (M*etaDotDot_r + C*etaDot_r - tau_est - Kd*v_eta - Kp*e);
    geometry_msgs::Wrench msg;
    tf::wrenchEigenToMsg(wrench, msg);
    il_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inner_loop_node");

    ros::NodeHandle nh("~");

    Kp  = nh.param<double>("Kp", 1) * Eigen::Matrix3d::Identity();
    Kd  = nh.param<double>("Kd", 1) * Eigen::Matrix3d::Identity();
    m   = nh.param<double>("mass", 0.1);
    I = Eigen::Vector3d(nh.param<double>("Ixx", 0.1),
                        nh.param<double>("Iyy", 0.1),
                        nh.param<double>("Izz", 0.1)).asDiagonal();

    v = nh.param<double>("v", 1);

    il_sub  = nh.subscribe("/sub_topic", 1, odom_cb);
    il_pub  = nh.advertise<geometry_msgs::Wrench>("/pub_topic", 1);
    
    ros::spin();

    return 0;
}