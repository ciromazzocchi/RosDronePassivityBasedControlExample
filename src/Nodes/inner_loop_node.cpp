#include "ros/ros.h"

#include "../Utils/utility.hpp"
#include "quad_control/UavState.h"
#include "nav_msgs/Odometry.h"

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

ros::Subscriber il_odom_sub, il_wrench_est_sub, il_eta_d_sub, il_mu_hat_sub;
ros::Publisher  il_cmd_pub;
ros::Publisher  il_e_eta_pub;
ros::Publisher  il_e_eta_d_pub;

double m, psi_d, v, uT;
Eigen::Vector3d eta, eta_dot, tau_est;
Eigen::Vector3d eta_d,eta_dot_d,eta_dot_dot_d;
Eigen::Matrix3d Kp, Kd, I;

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    eta <<  odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z;
    
    eta_dot <<  odom_msg->twist.twist.angular.x,
        odom_msg->twist.twist.angular.y,
        odom_msg->twist.twist.angular.z;

    Eigen::Vector3d e         = eta         - eta_d;
    Eigen::Vector3d e_dot     = eta_dot     - eta_dot_d;

    Eigen::Vector3d eta_r_dot     = eta_dot_d - v*e;
    Eigen::Vector3d eta_r_dot_dot = eta_dot_dot_d - v*e_dot;
    Eigen::Vector3d v_eta = e_dot + v*eta;

    Eigen::Matrix3d Q = getQ(eta);
    Eigen::Vector3d tau_b = Q.transpose().inverse() * (
        M(eta,I)*eta_r_dot_dot + C(eta,eta_dot, I)*eta_r_dot - tau_est - Kd*v_eta - Kp*e
    );

//    geometry_msgs::Wrench msg;
//    tf::vectorEigenToMsg(Eigen::Vector3d(0,0,uT), msg.force);
//    tf::vectorEigenToMsg(tau_b, msg.torque);
//    il_cmd_pub.publish(msg);
    geometry_msgs::Vector3Stamped vec_msg;
    vec_msg.header = odom_msg->header;
    vec_msg.vector.x = e.x();
    vec_msg.vector.y = e.y();
    vec_msg.vector.z = e.z();
    il_e_eta_pub.publish(vec_msg.vector);
    
    vec_msg.vector.x = e_dot.x();
    vec_msg.vector.y = e_dot.y();
    vec_msg.vector.z = e_dot.z();
    il_e_eta_d_pub.publish(vec_msg.vector);
}

void wrench_est_cb(const geometry_msgs::Wrench::ConstPtr& tau_msg) {
    tau_est << tau_msg->torque.x, tau_msg->torque.y, tau_msg->torque.z;
}

void eta_d_cb(const quad_control::UavState::ConstPtr& traj_msg) {
    eta_d << traj_msg->pose.x,
        traj_msg->pose.y,
        traj_msg->pose.z;

    eta_dot_d << traj_msg->twist.x,
            traj_msg->twist.y,
            traj_msg->twist.z;
    
    eta_dot_dot_d << traj_msg->wrench.x,
            traj_msg->wrench.y,
            traj_msg->wrench.z;
}

void mu_hat_cb(const geometry_msgs::Vector3Stamped::ConstPtr& mu_hat_msg) {
    Eigen::Vector3d mu_hat = Eigen::Vector3d( mu_hat_msg->vector.x,
                                    mu_hat_msg->vector.y, mu_hat_msg->vector.z);

    uT = m*sqrt( mu_hat.transpose() * mu_hat);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inner_loop_node");
    ros::NodeHandle nh("~");

    double Ts = nh.param<double>("Ts", 0.001);
    ros::Rate rate(1/Ts);

    Kp  = nh.param<double>("Kp", 1) * Eigen::Matrix3d::Identity();
    Kd  = nh.param<double>("Kd", 1) * Eigen::Matrix3d::Identity();
    m   = nh.param<double>("mass", 0.1);
    I = Eigen::Vector3d(nh.param<double>("Ixx", 0.1),
                        nh.param<double>("Iyy", 0.1),
                        nh.param<double>("Izz", 0.1)).asDiagonal();
    tau_est     = Eigen::Vector3d::Zero();
    psi_d = 0;

    eta_d           = Eigen::Vector3d::Zero();
    eta_dot_d       = Eigen::Vector3d::Zero();
    eta_dot_dot_d   = Eigen::Vector3d::Zero();

    eta             = Eigen::Vector3d::Zero();
    eta_dot         = Eigen::Vector3d::Zero();

    v = nh.param<double>("v", 10);

    il_odom_sub         = nh.subscribe("/odometry", 1, odom_cb);
    il_wrench_est_sub   = nh.subscribe("/wrenchEstimator", 1, wrench_est_cb);
    il_eta_d_sub        = nh.subscribe("/eta_d", 1, eta_d_cb);
    il_mu_hat_sub       = nh.subscribe("/mu_hat", 1, mu_hat_cb);
    il_cmd_pub          = nh.advertise<geometry_msgs::Wrench>("/wrenchNED", 1);
    il_e_eta_pub        = nh.advertise<geometry_msgs::Vector3Stamped>("/error_pose", 1);
    il_e_eta_d_pub      = nh.advertise<geometry_msgs::Vector3Stamped>("/error_twist", 1);
    
    ros::Duration(0.1).sleep();

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}