#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

ros::Subscriber odometry_sub;
ros::Publisher  odometry_pub;

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

    
    nav_msgs::Odometry msg;
    
    msg.header = odom->header;

    msg.pose.pose.position.x = p.x();
    msg.pose.pose.position.y = p.y();
    msg.pose.pose.position.z = p.z();

    msg.twist.twist.linear.x = pDot.x();
    msg.twist.twist.linear.y = pDot.y();
    msg.twist.twist.linear.z = pDot.z();

    msg.pose.pose.orientation.x = eta.x();
    msg.pose.pose.orientation.y = eta.y();
    msg.pose.pose.orientation.z = eta.z();
    msg.pose.pose.orientation.w = 0;

    msg.twist.twist.angular.x = etaDot.x();
    msg.twist.twist.angular.y = etaDot.y();
    msg.twist.twist.angular.z = etaDot.z();

    odometry_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometryNode");

    ros::NodeHandle nh("~");

    double Ts = nh.param<double>("Ts", 0.001);
    ros::Rate rate(1/Ts);

    odometry_sub    = nh.subscribe("/odometry", 1, odomCallback);
    odometry_pub    = nh.advertise<nav_msgs::Odometry>("/odometryNED", 1);
    
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}