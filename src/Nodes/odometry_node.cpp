#include "ros/ros.h"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include "nav_msgs/Odometry.h"
#include "quad_control/State.h"

#include "../Utils/utility.hpp"

ros::Subscriber odom_sub;
ros::Publisher  odom_p_pub, odom_eta_pub;

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom) {
    
    /* Creates a copu of odometry message */
    quad_control::State msg;

    /* Calculates the Rotation matrix from the  quaternion */
    Eigen::Quaterniond q = Eigen::Quaterniond( odom->pose.pose.orientation.w,
        odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z);

    Eigen::Matrix3d RbNed = q.normalized().toRotationMatrix();
    
    /* Calculates uav state in base NED Frame*/
    Eigen::Vector3d p = Eigen::Vector3d( odom->pose.pose.position.x,
        odom->pose.pose.position.y, odom->pose.pose.position.z );

    Eigen::Vector3d eta = getEta(q);

    Eigen::Vector3d p_dot = RbNed*Eigen::Vector3d( odom->twist.twist.linear.x,
        odom->twist.twist.linear.y, odom->twist.twist.linear.z);

    Eigen::Vector3d eta_dot = getQ(eta).inverse()*Eigen::Vector3d ( 
        odom->twist.twist.angular.x, odom->twist.twist.angular.y,
        odom->twist.twist.angular.z);
    
    /* Creates position message */
    tf::vectorEigenToMsg(p,msg.position);
    tf::vectorEigenToMsg(p_dot,msg.speed);
    odom_p_pub.publish(msg);

    /* Creates orientation message */
    tf::vectorEigenToMsg(eta,msg.position);
    tf::vectorEigenToMsg(eta_dot,msg.speed);
    odom_eta_pub.publish(msg);
}

int main(int argc, char **argv)
{
    /* Initializes node */
    ros::init(argc, argv, "odometry_node");

    /* Initializes node handler */
    ros::NodeHandle nh("~");

    /* Initializes subscriber to receive odometry messages from NED plguin */
    odom_sub = nh.subscribe("/odometry", 1, odom_cb);

    /* Publishes states in world NED frame */
    odom_p_pub      = nh.advertise<quad_control::State>("/p", 1);
    odom_eta_pub    = nh.advertise<quad_control::State>("/eta", 1);

    /* Initializes rate to x frequency */
    ros::Rate rate( nh.param<double>("/rate", 100) );

    /* Starts */
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}