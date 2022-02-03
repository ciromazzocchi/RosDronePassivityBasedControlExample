#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "quad_control/UavState.h"
#include "../Utils/Filter/Differentiator.hpp"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

quad_control::UavState msg;
ros::Subscriber odomSub;
ros::Publisher odomPub;

Differentiator<Eigen::Vector3d> pFilter;
Differentiator<Eigen::Vector3d> etaFilter;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    
    tf::Matrix3x3 RNed;
    RNed.setEulerYPR(M_PI/2,0,M_PI);

    tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,  odom->pose.pose.orientation.w);
    tf::Matrix3x3 Rb(q);
    tf::Matrix3x3 RbNed = RNed*Rb*(RNed.transpose());

    Eigen::Matrix3d Rned;
    Rned << RNed[0][0], RNed[0][1], RNed[0][2],
            RNed[1][0], RNed[1][1], RNed[1][2],
            RNed[2][0], RNed[2][1], RNed[2][2];

    Eigen::Vector3d p = Eigen::Vector3d( odom->pose.pose.position.x,
        odom->pose.pose.position.y, odom->pose.pose.position.z);
    p = Rned*p;

    Eigen::Vector3d eta;
    RbNed.getRPY(eta[0],eta[1],eta[2]);

    Eigen::Vector3d pDot = Eigen::Vector3d( odom->twist.twist.linear.x,
        odom->twist.twist.linear.y, odom->twist.twist.linear.z);
    pDot = Rned*pDot;

    Eigen::Vector3d etaDot = Eigen::Vector3d ( odom->twist.twist.angular.x,
        odom->twist.twist.angular.y, odom->twist.twist.angular.z);
    etaDot = Rned*etaDot;

    Eigen::Vector3d pDotDot     = pFilter.getDifferentiatoredValue(pDot);
    Eigen::Vector3d etaDotDot   = etaFilter.getDifferentiatoredValue(etaDot);

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "worldNED";

    tf::vectorEigenToMsg(p,msg.pose.linear);
    tf::vectorEigenToMsg(pDot,msg.twist.linear);
    tf::vectorEigenToMsg(pDotDot,msg.wrench.linear);
    tf::vectorEigenToMsg(eta,msg.pose.angular);
    tf::vectorEigenToMsg(etaDot,msg.twist.angular);
    tf::vectorEigenToMsg(etaDotDot,msg.wrench.angular);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometryNode");
    ros::NodeHandle nh("~");

    double kf = nh.param<double>("kf", 100);
    double Ts = nh.param<double>("Ts", 0.001);
    ros::Rate rate(1/Ts);

    odomSub = nh.subscribe("/odometry", 1, odomCallback);
    odomPub = nh.advertise<quad_control::UavState>("/odometryNED", 1);

    pFilter.DifferentiatorInit( kf, Ts );
    etaFilter.DifferentiatorInit( kf, Ts );
    
    while(ros::ok()) {
        ros::spinOnce();
        msg.header.stamp = ros::Time::now();
        odomPub.publish(msg);
        rate.sleep();
    }

    return 0;
}