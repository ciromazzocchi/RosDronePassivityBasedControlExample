#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "quad_control/UavState.h"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

ros::Subscriber odometry_sub;
ros::Publisher  odometry_pub;

Eigen::Matrix<double,6,1> pose, twist; 

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    
    Eigen::Quaterniond q = Eigen::Quaterniond( odom->pose.pose.orientation.w,
        odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z);
    Eigen::Matrix3d RbNed = q.normalized().toRotationMatrix().transpose();

    pose    <<  odom->pose.pose.position.x, odom->pose.pose.position.y,
                odom->pose.pose.position.z, RbNed.eulerAngles(0,1,2);

    twist   <<  RbNed*Eigen::Vector3d( odom->twist.twist.linear.x,
                    odom->twist.twist.linear.y, odom->twist.twist.linear.z),
                RbNed*Eigen::Vector3d ( odom->twist.twist.angular.x,
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

    odometry_sub    = nh.subscribe("/gazebo_odometry", 1, odomCallback);
    odometry_pub    = nh.advertise<quad_control::UavState>("/odometry", 1);
    
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}