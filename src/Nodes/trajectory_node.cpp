#include "ros/ros.h"

#include <list>

#include <Eigen/Dense>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include "../Utils/Differentiator.hpp"
#include "../Utils/utility.hpp"

#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "quad_control/State.h"

using namespace Eigen;

std::list<Vector3d> waypoints_list;

std::list<Vector3d>::iterator actual_point;

Vector3d p, p_dot, f_rep;

double threshold, ka, rate, psi, k_rep, rng_infl;

Differentiator<Vector3d> acceleration_filter;

ros::Subscriber traj_sub, gaz_sub;
ros::Publisher  traj_p_pub, traj_yaw_pub;

Matrix3d R_NED;

void gaz_cb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    f_rep = Vector3d::Zero();

    for(int i=0; i< msg->name.size(); i++) {
        if((msg->name[i] != "ground_plane") && (msg->name[i] != "hummingbird")) {
            Vector3d obj = R_NED * Vector3d(msg->pose[i].position.x,
                msg->pose[i].position.y, msg->pose[i].position.z);
            
            Vector2d obj2 = Vector2d(obj.x(), obj.y());
            Vector2d p2   = Vector2d(  p.x(),   p.y());
            
            double dist2 = (p2 - obj2).norm();
            double dist =  (p - obj).norm();
            if( dist2 <= rng_infl) {
                double c = (k_rep / (p2 - obj2).squaredNorm()) * pow(1/dist2 - 1/rng_infl, 1);
                Vector3d f_temp;
                f_temp << c * (p2 - obj2).normalized(), 0;
                f_rep = f_rep + f_temp;
            }
        }
    }
    //ROS_ERROR_STREAM(f_rep.transpose());
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg) {

    Quaterniond q = Quaterniond( odom_msg->pose.pose.orientation.w,
        odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z);
    
    Matrix3d RbNed = q.normalized().toRotationMatrix();

    p = Vector3d( odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z );

    p_dot = RbNed*Vector3d( odom_msg->twist.twist.linear.x,
        odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.z);

    Vector3d error = waypoints_list.front()-p;
    
    if(error.norm() < threshold & p_dot.norm() < 0.5 & (waypoints_list.size()>1)) 
        waypoints_list.pop_front();

    //ROS_ERROR_STREAM("n_waypoints" << waypoints_list.size());
    Vector3d position, speed, acceleration;
    speed = error * ka / (error.norm() > 1 ? error.norm() : 1 );
    speed = speed + f_rep;
    acceleration = acceleration_filter.getDifferentiatoredValue(speed);
    position = p + speed/rate;

    if(error.x() >= 0.01)
        psi = atan2(p.y(),p.x()) - M_PI_2 ;

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

    psi = 0;

    threshold   = nh.param<double>("threshold", 0.01);
    ka          = nh.param<double>("ka", 10);
    rate        = nh.param<double>("/rate", 1000);
    double kf_d = nh.param<double>("kf_d", 100);
    rng_infl    = nh.param<double>("/rng_infl", 1);
    k_rep       = nh.param<double>("/k_rep", 1);

    acceleration_filter.DifferentiatorInit( kf_d, 1/rate );

    waypoints_list.push_back(Vector3d(0,0,-1));
    waypoints_list.push_back(Vector3d(7,7,-2));
    waypoints_list.push_back(Vector3d(7,7,-0.06));

    actual_point = waypoints_list.begin();

    gaz_sub      = nh.subscribe("/gazebo", 1, gaz_cb);
    traj_sub     = nh.subscribe("/p", 1, odom_cb);
    traj_p_pub   = nh.advertise<quad_control::State>("/traj/p", 1);
    traj_yaw_pub = nh.advertise<std_msgs::Float64>("/traj/yaw", 1);
    
    tf::Matrix3x3 RNed;
    RNed.setEulerYPR(M_PI/2,0,M_PI);
    tf::matrixTFToEigen(RNed,R_NED);

    ros::spin();

    return 0;
}