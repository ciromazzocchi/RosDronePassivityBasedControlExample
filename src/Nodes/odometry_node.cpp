#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "quad_control/UavState.h"
#include "../Utils/Differentiator.hpp"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

ros::Subscriber odometry_sub;
ros::Publisher odometry_p_pub, odometry_eta_pub;

Differentiator<Eigen::Vector3d> pFilter;
Differentiator<Eigen::Vector3d> etaFilter;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    
    //RNed.setEulerYPR(M_PI/2,0,M_PI);
    Eigen::Quaterniond q;
    q.x() = odom->pose.pose.orientation.x;
    q.y() = odom->pose.pose.orientation.y;
    q.z() = odom->pose.pose.orientation.z;
    q.w() = odom->pose.pose.orientation.w;
    Eigen::Matrix3d RbNed = q.normalized().toRotationMatrix();

    Eigen::Vector3d p = Eigen::Vector3d(        odom->pose.pose.position.x,
                                                odom->pose.pose.position.y,
                                                odom->pose.pose.position.z);

    Eigen::Vector3d eta = RbNed.eulerAngles(2,1,0);

    Eigen::Vector3d pDot = Eigen::Vector3d(     odom->twist.twist.linear.x,
                                                odom->twist.twist.linear.y,
                                                odom->twist.twist.linear.z);
    pDot = RbNed*pDot;

    Eigen::Vector3d etaDot = Eigen::Vector3d (  odom->twist.twist.angular.x,
                                                odom->twist.twist.angular.y,
                                                odom->twist.twist.angular.z);
    etaDot = RbNed*etaDot;

    Eigen::Vector3d pDotDot     = pFilter.getDifferentiatoredValue(pDot);
    Eigen::Vector3d etaDotDot   = etaFilter.getDifferentiatoredValue(etaDot);

    quad_control::UavState msg;
    
    msg.header = odom->header;
    tf::vectorEigenToMsg(p,msg.pose);
    tf::vectorEigenToMsg(pDot,msg.twist);
    tf::vectorEigenToMsg(pDotDot,msg.wrench);
    odometry_p_pub.publish(msg);

    tf::vectorEigenToMsg(eta,msg.pose);
    tf::vectorEigenToMsg(etaDot,msg.twist);
    tf::vectorEigenToMsg(etaDotDot,msg.wrench);
    odometry_eta_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometryNode");

    ros::NodeHandle nh("~");

    double kf = nh.param<double>("kf", 100);
    double Ts = nh.param<double>("Ts", 0.001);
    ros::Rate rate(1/Ts);

    odometry_sub     = nh.subscribe("/odometry", 1, odomCallback);
    odometry_eta_pub = nh.advertise<quad_control::UavState>("/etaNED", 1);
    odometry_p_pub   = nh.advertise<quad_control::UavState>("/pNED", 1);
    pFilter.DifferentiatorInit( kf, Ts );
    etaFilter.DifferentiatorInit( kf, Ts );
    
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}