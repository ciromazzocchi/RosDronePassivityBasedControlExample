#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "quad_control/UavState.h"
#include "../Utils/utility.hpp"
#include "../Utils/Differentiator.hpp"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

ros::Subscriber odometry_sub;
ros::Publisher  odometry_pub;

Eigen::Matrix<double,6,1> pose, twist; 

Differentiator<Eigen::Matrix<double,6,1>> twistDifferentiator;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    
    Eigen::Quaterniond q = Eigen::Quaterniond( odom->pose.pose.orientation.w,
        odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z);
    
    //Eigen::Matrix3d RbNed = q.normalized().toRotationMatrix().inverse();

    Eigen::Vector3d eta = getEta(q);

    Eigen::Matrix3d RbNed = getR(eta).inverse();
    // Eigen::Vector3d eta = RbNed.eulerAngles(2,1,0);
    // for(int i=0; i < 3; i++) {
    //     if(eta(i) >= 0)
    //         eta(i) = eta(i) - M_PI;
    //     else
    //         eta(i) = eta(i) + M_PI;
    // }

    pose    <<  odom->pose.pose.position.x, odom->pose.pose.position.y,
                odom->pose.pose.position.z, eta;

    //twist = twistDifferentiator.getDifferentiatoredValue(pose);
   twist   <<  getR(eta)*Eigen::Vector3d( odom->twist.twist.linear.x,
                    odom->twist.twist.linear.y, odom->twist.twist.linear.z),
                getR(eta)*Eigen::Vector3d ( odom->twist.twist.angular.x,
                    odom->twist.twist.angular.y, odom->twist.twist.angular.z);
    
    quad_control::UavState msg;
    msg.header = odom->header;
    tf::twistEigenToMsg(pose,msg.pose);
    tf::twistEigenToMsg(twist,msg.twist);
    odometry_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_node");

    ros::NodeHandle nh("~");

    ros::Rate rate(1000);

    twistDifferentiator.DifferentiatorInit(100,0.001);

    odometry_sub    = nh.subscribe("/gazebo_odometry", 1, odomCallback);
    odometry_pub    = nh.advertise<quad_control::UavState>("/odometry", 1);
    
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}