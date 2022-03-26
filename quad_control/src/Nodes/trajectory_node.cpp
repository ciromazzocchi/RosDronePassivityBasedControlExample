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
std::list<Vector3d> skeleton_point;
std::list<Vector3d> trapping_point;
std::list<Vector3d> object_point;

std::list<Vector3d>::iterator actual_point;

Vector3d p, p_dot;

double threshold, ka, rate, psi, k_rep, rng_infl, k_psi;
double T_local;
bool flag_local_minimum;

Differentiator<Vector3d> acceleration_filter;

ros::Subscriber traj_sub, gaz_sub;
ros::Publisher  traj_p_pub, traj_yaw_pub;

Matrix3d R_NED;

Vector3d getFAtt(Vector3d p_uav) {
    Vector3d error = waypoints_list.front()-p_uav;
    Vector3d f_att = error * ka / (error.norm() > 1 ? error.norm() : 1 );
    return f_att;
}

Vector3d getFRep (Vector3d p_uav) {
    Vector3d f_rep = Vector3d::Zero();
    for(std::list<Vector3d>::iterator it = object_point.begin();
            it != object_point.end(); it++) {
        
        Vector2d obj2 = Vector2d(it->x(), it->y());
        Vector2d p2   = Vector2d(p_uav.x(), p_uav.y());
            
        double dist2 = (p2 - obj2).norm();
        double dist =  (p_uav - *it).norm();
        double c = (k_rep / (p2 - obj2).squaredNorm()) * pow(1/dist2 - 1/rng_infl, 1);
        Vector3d f_temp;
        f_temp << c * (p2 - obj2).normalized(), 0;
        f_rep = f_rep + f_temp;
    }
    return f_rep;
}

/* TO DO: defines the direction of virtual point as orthogonal direction of drone */
Vector3d getVirtualPoint(Vector3d p_uav) {
    Vector3d f_virtual_point = Vector3d::Zero();

    for(std::list<Vector3d>::iterator it = trapping_point.begin();
            it != trapping_point.end(); it++) {
        Vector2d obj2 = Vector2d(it->x(), it->y());
        Vector2d p2   = Vector2d(p_uav.x(), p_uav.y());
         
        double dist2 = (p2 - obj2).norm();

        if( dist2 <= rng_infl) {
            Vector3d y = Vector3d(0,0,1);
            Vector3d x = getFAtt(p).normalized();

            Vector3d f_temp = 0.03 * k_rep * y.cross(x);
            f_virtual_point = f_virtual_point - f_temp;
        }
    }

    for(std::list<Vector3d>::iterator it = trapping_point.begin();
            it != trapping_point.end(); it++) {
        Vector2d obj2 = Vector2d(it->x(), it->y());
        Vector2d p2   = Vector2d(p_uav.x(), p_uav.y());
         
        double dist2 = (p2 - obj2).norm();

        if( dist2 > rng_infl/2) {
            trapping_point.erase(it--);
        }
    }

    return f_virtual_point;
}

Vector3d add_trapping_point() {
    double max = 0;
    Vector3d max_p;
    for( std::list<Vector3d>::iterator it=skeleton_point.begin();
          it != skeleton_point.end(); it++) {
        Vector3d f_att = getFAtt(*it);
        Vector3d f_rep = getFRep(*it);
        double dot_prod = f_att.transpose() * (-f_rep);

        if(dot_prod > max) {
            max = dot_prod;
            max_p = *it;
        }
    }
    trapping_point.push_back(max_p);
}

void gaz_cb(const gazebo_msgs::ModelStates::ConstPtr& msg) { 
    object_point.erase(object_point.begin(), object_point.end());
    for(int i=0; i< msg->name.size(); i++) {
        if((msg->name[i] != "ground_plane") && (msg->name[i] != "hummingbird")) {
            Vector3d obj = R_NED * Vector3d(msg->pose[i].position.x,
                msg->pose[i].position.y, msg->pose[i].position.z);
            
            Vector2d obj2 = Vector2d(obj.x(), obj.y());
            Vector2d p2   = Vector2d(  p.x(),   p.y());
            
            double dist2 = (p2 - obj2).norm();
            double dist =  (p - obj).norm();
            if( dist2 <= rng_infl) {
                object_point.push_back(obj);
            }
        }
    }
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

    /* Creatin skeleton point */
    skeleton_point.erase(skeleton_point.begin(),skeleton_point.end());
    for(double i=0; i<2*M_PI; i=i+M_PI/4 ) {
        Vector3d p_o = Vector3d( 0.5 * cos(i), 0.5 * sin(i), 0  );
        skeleton_point.push_front(p + p_o);
    }

    /* Defining distance between desired position and actual position */
    Vector3d error = waypoints_list.front()-p;
    
    if(error.norm() < threshold & p_dot.norm() < 0.5 & (waypoints_list.size()>1)) 
        waypoints_list.pop_front();

    if(error.norm() > threshold & p_dot.norm() < 0.05) {
        T_local = T_local + 1/rate;
        ROS_ERROR_STREAM("T_local " << T_local << " trapping point " << trapping_point.size());
        if(T_local > 1) {
            add_trapping_point();
            T_local = 0;
        }
    } else {
        T_local = 0;
    }
    Vector3d position, speed, acceleration;
    
    speed = getFAtt(p) + getFRep(p) + getVirtualPoint(p);

    acceleration = acceleration_filter.getDifferentiatoredValue(speed);
    position = p + speed/rate;

    //if(error.x() >= 0.01)
    //    psi = atan2(p.y(),p.x()) - M_PI_2 ;

    if(error.x() >= 0.001) {
        //psi = (1-k_psi/rate)*psi  - (k_psi/rate)*( atan2(error.y(),error.x()));
        psi = (1-k_psi/rate)*psi  - (k_psi/rate)*( atan2(error.x(),error.y()));
    }

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

    threshold   = nh.param<double>("/threshold", 0.01);
    ka          = nh.param<double>("/ka", 10);
    rate        = nh.param<double>("/rate", 1000);
    double kf_d = nh.param<double>("/pl_kf_d", 100);
    rng_infl    = nh.param<double>("/rng_infl", 1);
    k_rep       = nh.param<double>("/k_rep", 1);
    k_psi       = nh.param<double>("/k_psi", 1);


    T_local = 0;
    flag_local_minimum = false;
    
    acceleration_filter.DifferentiatorInit( kf_d, 1/rate );

    waypoints_list.push_back(Vector3d(0,0,-1));
    waypoints_list.push_back(Vector3d(10,10,-2));
    waypoints_list.push_back(Vector3d(10,10,-0.06));

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