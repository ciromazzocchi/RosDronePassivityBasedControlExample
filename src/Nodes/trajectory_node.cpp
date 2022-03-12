#include "ros/ros.h"

#include <fstream>
#include <list>

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include "../Utils/Differentiator.hpp"
#include "../Utils/utility.hpp"

#include "std_msgs/Float64.h"
#include "quad_control/State.h"

std::list<Eigen::Vector3d> waypoints_list;

std::list<Eigen::Vector3d>::iterator actual_point;

double threshold, ka, rate;

Differentiator<Eigen::Vector3d> acceleration_filter;

ros::Subscriber traj_sub;
ros::Publisher  traj_p_pub, traj_yaw_pub;


void odom_cb(const quad_control::State::ConstPtr& odom_msg) {

    Eigen::Vector3d p = Eigen::Vector3d(odom_msg->position.x,
        odom_msg->position.y, odom_msg->position.z);
    Eigen::Vector3d p_dot = Eigen::Vector3d(odom_msg->speed.x,
        odom_msg->speed.y, odom_msg->speed.z);

    Eigen::Vector3d error = waypoints_list.front()-p;
    
    if(error.norm() < threshold & p_dot.norm() < 0.5 & (waypoints_list.size()>1)) 
        waypoints_list.pop_front();

    Eigen::Vector3d position, speed, acceleration;
    speed = error * ka / (error.norm() > 1 ? error.norm() : 1 );
    acceleration = acceleration_filter.getDifferentiatoredValue(speed);
    position = p + speed/rate;


    double psi = 0;

    quad_control::State msg1;
    tf::vectorEigenToMsg(position,      msg1.position);
    tf::vectorEigenToMsg(speed,         msg1.speed);
    tf::vectorEigenToMsg(acceleration,  msg1.acceleration);
    traj_p_pub.publish(msg1);

    std_msgs::Float64 msg2;
    msg2.data = psi;
    traj_yaw_pub.publish(msg2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh("~");

    threshold   = nh.param<double>("threshold", 0.01);
    ka          = nh.param<double>("ka", 10);
    rate        = nh.param<double>("/rate", 1000);
    double kf_d = nh.param<double>("kf_d", 100);

    acceleration_filter.DifferentiatorInit( kf_d, 1/rate );

    waypoints_list.push_back(Eigen::Vector3d(0,0,-1));
    waypoints_list.push_back(Eigen::Vector3d(7,7,-2));
    waypoints_list.push_back(Eigen::Vector3d(7,7,-0.06));

    actual_point = waypoints_list.begin();

    traj_sub     = nh.subscribe("/p", 1, odom_cb);
    traj_p_pub   = nh.advertise<quad_control::State>("/traj/p", 1);
    traj_yaw_pub = nh.advertise<std_msgs::Float64>("/traj/yaw", 1);
    
    ros::spin();

    return 0;
}