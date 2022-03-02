#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "../Utils/utility.hpp"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

ros::Subscriber odometry_sub;
ros::Publisher  odometry_pub;

Eigen::Matrix<double,6,1> twist; 

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    
    Eigen::Quaterniond q = Eigen::Quaterniond( odom->pose.pose.orientation.w,
        odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z);
    
    Eigen::Matrix3d RbNed = q.normalized().toRotationMatrix();
    
    Eigen::Vector3d eta = getEta(q);

    twist   <<  RbNed*Eigen::Vector3d( odom->twist.twist.linear.x,
                    odom->twist.twist.linear.y, odom->twist.twist.linear.z),
                getQ(eta).inverse()*Eigen::Vector3d ( odom->twist.twist.angular.x,
                    odom->twist.twist.angular.y, odom->twist.twist.angular.z);
    
    nav_msgs::Odometry msg;
    msg = *odom;
    tf::twistEigenToMsg(twist,msg.twist.twist);
    odometry_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_node");

    ros::NodeHandle nh("~");

    odometry_sub    = nh.subscribe("/sub_topic", 1, odomCallback);
    odometry_pub    = nh.advertise<nav_msgs::Odometry>("/pub_topic", 1);
    
    ros::Rate rate(100);
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}