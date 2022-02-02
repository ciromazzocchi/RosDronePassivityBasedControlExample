#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "quad_control/UavState.h"
#include "../Utils/Filter/LP2Filter.h"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

quad_control::UavState msg;
ros::Subscriber odomSub;
ros::Publisher odomPub;

LP2Filter<Eigen::Vector3d> pFilter;
LP2Filter<Eigen::Vector3d> etaFilter;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    
    tf::Matrix3x3 RNed;
    RNed.setEulerYPR(M_PI/2,0,M_PI);
    tf::Vector3 p;

    tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,  odom->pose.pose.orientation.w);
    tf::Matrix3x3 Rb(q);
    tf::Matrix3x3 RbNed = RNed*Rb*(RNed.transpose());

    p[0] = odom->pose.pose.position.x;
    p[1] = odom->pose.pose.position.y;
    p[2] = odom->pose.pose.position.z;

    p = RNed*p;

    Eigen::Vector3d eta; RbNed.getRPY(eta[0],eta[1],eta[2]);

    pFilter.filterStep(Eigen::Vector3d(p[0], p[1], p[2]));
    etaFilter.filterStep(eta);

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "worldNED";

    tf::vectorEigenToMsg(pFilter.lastFiltered(),msg.pose.linear);
    tf::vectorEigenToMsg(pFilter.lastFirst(),msg.twist.linear);
    tf::vectorEigenToMsg(pFilter.lastSecond(),msg.wrench.linear);
    tf::vectorEigenToMsg(etaFilter.lastFiltered(),msg.pose.angular);
    tf::vectorEigenToMsg(etaFilter.lastFirst(),msg.twist.angular);
    tf::vectorEigenToMsg(etaFilter.lastSecond(),msg.wrench.angular);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometryNode");
    ros::NodeHandle nh("~");
    ros::Rate rate(1000);

    odomSub = nh.subscribe("/odometry", 1, odomCallback);
    odomPub = nh.advertise<quad_control::UavState>("/odometryNED", 1);

    pFilter.initFilterStep( 0.001, 100, 100, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    etaFilter.initFilterStep( 0.001, 100, 100, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    
    while(ros::ok()) {
        ros::spinOnce();
        msg.header.stamp = ros::Time::now();
        odomPub.publish(msg);
        rate.sleep();
    }

    return 0;
}