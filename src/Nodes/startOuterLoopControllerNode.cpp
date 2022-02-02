#include "ros/ros.h"
#include "quad_control/UavState.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <Eigen/Dense>

Eigen::Vector3d p, pDot, pDotDot;
Eigen::Vector3d p_d, pDot_d, pDotDot_d;
geometry_msgs::Vector3Stamped msg;
bool new_odom;

void odomCallback(const quad_control::UavState::ConstPtr& odom){
    p << odom->pose.linear.x, odom->pose.linear.y, odom->pose.linear.z;
    pDot << odom->twist.linear.x, odom->twist.linear.y, odom->twist.linear.z;
    pDotDot << odom->wrench.linear.x, odom->wrench.linear.y, odom->wrench.linear.z;
    new_odom = true;
};

void trajCallback(const quad_control::UavState::ConstPtr& traj){
    p_d << traj->pose.linear.x, traj->pose.linear.y, traj->pose.linear.z;
    pDot_d << traj->twist.linear.x, traj->twist.linear.y, traj->twist.linear.z;
    pDotDot_d << traj->wrench.linear.x, traj->wrench.linear.y, traj->wrench.linear.z;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "OuterloopController");
    ros::NodeHandle nh("~");

    Eigen::Vector3d e, eDot, eInt;
    e = eDot = eInt = Eigen::Vector3d::Zero();
    
    p_d = Eigen::Vector3d(0, 0, -0.06);
    pDot_d = pDotDot_d = Eigen::Vector3d::Zero();
    
    Eigen::Vector3d mu = Eigen::Vector3d::Zero();

    double fc = nh.param<double>("rate", 1000);
    ros::Rate rate( fc );

    Eigen::Matrix3d Kp, Kd, Ki;
    Kp = nh.param<double>("Kp", 1) * Eigen::Matrix3d::Identity();
    Kd = nh.param<double>("Kd", 1) * Eigen::Matrix3d::Identity();
    Ki = nh.param<double>("Ki", 1) * Eigen::Matrix3d::Identity();

    ros::Subscriber odomSub = nh.subscribe("/odometry", 1, odomCallback);
    ros::Subscriber trajSub = nh.subscribe("/trajectory", 1, trajCallback);

    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3Stamped>("/desiredMu", 1);

    while(ros::ok()) {
        ros::spinOnce();
        if(new_odom) {
            msg.header.stamp = ros::Time::now();

            e = p - p_d;
            eDot = pDot - pDot_d;

            eInt = eInt + e/fc;

            mu = pDotDot - ( Kp * e + Kd * eDot + Ki * eInt );

            msg.vector.x = mu.x();
            msg.vector.y = mu.y();
            msg.vector.z = mu.z();

            pub.publish(msg);
        }
        rate.sleep();
    }

    return 0;
}