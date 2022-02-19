#include "ros/ros.h"
#include "quad_control/UavState.h"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

ros::Subscriber ol_sub;
ros::Publisher  ol_pub;

double m;
Eigen::Matrix3d Kp, Kd;

void odom_cb(const quad_control::UavState::ConstPtr& odom_msg) {
    Eigen::Vector3d pDotDot_d, e, eDot;

    tf::vectorMsgToEigen(odom_msg->wrench_d.linear,pDotDot_d);
    tf::vectorMsgToEigen(odom_msg->error_pose.linear,e);
    tf::vectorMsgToEigen(odom_msg->error_twist.linear,eDot);
    tf::vectorMsgToEigen(odom_msg->error_twist.linear,eDot);

    Eigen::Vector3d force_est = Eigen::Vector3d::Zero();
    
    Eigen::Vector3d mu = pDotDot_d - (1/m)*(Kp*e + Kd*eDot)  - (1/m)*force_est;

    if(mu[2] >  9) mu[2] =  9;
    if(mu[2] < -9) mu[2] = -9;

    quad_control::UavState msg;
    msg = *odom_msg;
    tf::vectorEigenToMsg(mu, msg.mu_hat);

    ol_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "outer_loop_node");
    ros::NodeHandle nh("~");

    ros::Rate rate(1000);

    Kp  = nh.param<double>("Kp", 1) * Eigen::Matrix3d::Identity();
    Kd  = nh.param<double>("Kd", 1) * Eigen::Matrix3d::Identity();
    m   = nh.param<double>("mass", 0.1);

    ol_sub = nh.subscribe("/trajectory", 1, odom_cb);
    ol_pub = nh.advertise<quad_control::UavState>("/mu_hat", 1);
    
    ros::Duration(0.01).sleep();

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}