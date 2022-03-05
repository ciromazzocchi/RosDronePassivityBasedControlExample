#include "ros/ros.h"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include "../Utils/utility.hpp"

#include "std_msgs/Float64.h"
#include "quad_control/State.h"

ros::Subscriber traj_sub;
ros::Publisher  traj_p_pub, traj_yaw_pub;

void odom_cb(const quad_control::State::ConstPtr& odom_msg) {
    quad_control::State msg1;
    std_msgs::Float64 msg2;

    if (odom_msg->position.z >= -1)
        msg1.position.z = odom_msg->position.z - 0.01;
    else
        msg1.position.z = -1;

    msg1.position.x = msg1.position.y = 0.0;

    msg1.speed.x  = msg1.speed.y  = msg1.speed.z  = 0.0;
    msg1.acceleration.x = msg1.acceleration.y = msg1.acceleration.z = 0.0;

    msg2.data = 0.0;

    traj_p_pub.publish(msg1);
    traj_yaw_pub.publish(msg2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh("~");

    traj_sub     = nh.subscribe("/p", 1, odom_cb);
    traj_p_pub   = nh.advertise<quad_control::State>("/traj/p", 1);
    traj_yaw_pub = nh.advertise<std_msgs::Float64>("/traj/yaw", 1);
    
    ros::spin();

    return 0;
}