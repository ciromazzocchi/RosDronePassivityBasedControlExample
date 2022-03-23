#include "ros/ros.h"

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include "quad_control/State.h"
#include "geometry_msgs/Wrench.h"

ros::Subscriber ol_odom_sub,ol_traj_sub, ol_est_sub;
ros::Publisher  ol_mu_pub;

double m;
Eigen::Vector3d p_d, p_dot_d, p_dot_dot_d, force_est;
Eigen::Matrix3d Kp, Kd;

void est_cb(const geometry_msgs::Wrench::ConstPtr& msg) {
    tf::vectorMsgToEigen(msg->force,force_est);
}

void traj_cb(const quad_control::State::ConstPtr& msg) {
    p_d         << msg->position.x, msg->position.y, msg->position.z;
    p_dot_d     << msg->speed.x, msg->speed.y, msg->speed.z;
    p_dot_dot_d << msg->acceleration.x, msg->acceleration.y, msg->acceleration.z; 
};

void odom_cb(const quad_control::State::ConstPtr& odom_msg) {
    Eigen::Vector3d p, p_dot;
    tf::vectorMsgToEigen( odom_msg->position ,p      );
    tf::vectorMsgToEigen( odom_msg->speed    ,p_dot  );

    Eigen::Vector3d e = p - p_d;
    Eigen::Vector3d e_dot = p_dot - p_dot_d;
    
    Eigen::Vector3d mu = p_dot_dot_d - (1/m)*(Kp*e + Kd*e_dot)  - (1/m)*force_est;

    if(mu[2] >  9) mu[2] =  9;
    if(mu[2] < -9) mu[2] = -9;

    geometry_msgs::Vector3 msg;
    tf::vectorEigenToMsg(mu,msg);
    ol_mu_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "outer_loop_node");
    ros::NodeHandle nh("~");

   
    m   = nh.param<double>("/mass", 0.1);

    double xi = nh.param<double>("/ol_xi",0.9);
    double wn = nh.param<double>("/ol_wn",25);

    Kp = 2*xi*wn   * m * Eigen::Matrix3d::Identity();
    Kd = pow(wn,2) * m * Eigen::Matrix3d::Identity();

    p_d = p_dot_d = p_dot_dot_d = force_est = Eigen::Vector3d::Zero();

    ol_odom_sub = nh.subscribe("/odom_topic", 1, odom_cb);
    ol_traj_sub = nh.subscribe("/traj_topic", 1, traj_cb);
    ol_est_sub  = nh.subscribe("/est_topic",  1, est_cb);
    ol_mu_pub   = nh.advertise<geometry_msgs::Vector3>("/mu_hat", 1);

    ros::spin();

    return 0;
}