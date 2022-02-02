#pragma once

#include <cmath>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>

#include "geometry_msgs/Wrench.h"
#include "quad_control/UavState.h"
#include "quad_control/DesiredTrajectory.h"
#include "quad_control/LogData.h"
#include "mav_msgs/Actuators.h"

#include "../Utils/Drone/Drone.hpp"
#include "../Utils/Filter/Filter.hpp"
#include "../Utils/Filter/LP2Filter.h"
#include "../Utils/Utility/utility.hpp"


/*
 * This is a passive controller for the VTOL UAV. Starting from desired
 * positions, velocities and accelerations, computes control wrenches.
 */
class Controller{

public:
    Controller();

    // Main loop
    void run();

private:
    ros::NodeHandle nh = ros::NodeHandle("~");

    double rate = nh.param<double>("rate", 1000);
    double Ts = 1 / rate;

    Matrix3d Rb;

    Drone UAV = Drone(nh.param<double>("m", 1), nh.param<double>("Ibx", 1),
        nh.param<double>("Iby", 1), nh.param<double>("Ibz", 1));

    // Passive Controller gains
    const Matrix3d Kp = Vector3d( nh.param<double>("Kpx", 10),
                                  nh.param<double>("Kpy", 10),
                                  nh.param<double>("Kpz", 10) ).asDiagonal();
    
    const Matrix3d Kd = Vector3d( nh.param<double>("Kdx", 1),
                                  nh.param<double>("Kdy", 1),
                                  nh.param<double>("Kdz", 1) ).asDiagonal();

    // Attitude passivity controller gains

    const Matrix3d D0 = Vector3d( nh.param<double>("D0x", 10),
                                  nh.param<double>("D0y", 10),
                                  nh.param<double>("D0z", 10) ).asDiagonal();
    
    const Matrix3d K0 = Vector3d( nh.param<double>("K0x", 1),
                                  nh.param<double>("K0y", 1),
                                  nh.param<double>("K0z", 1) ).asDiagonal();      
    
    const double v = nh.param<double>("v", 1.0);

    quad_control::UavState odomMsg;
    mav_msgs::Actuators    commMsg;

    double uT     = 0;
    Vector3d taub = Vector3d::Zero();
    Vector6d Fe   = Vector6d::Zero();

    ros::Time stamp;
    
    Filter<Vector6d> wrenchEstimatorFilter;
    LP2Filter<Vector3d> eta_filter;
    LP2Filter<Vector6d> poseFilter;

    ros::Subscriber subSensorRead     = nh.subscribe("/odometry",   1, &Controller::cbSensorRead,     this);
    ros::Subscriber subTrajectoryRead = nh.subscribe("/trajectory", 1, &Controller::cbTrajectoryRead, this);
    ros::Publisher  pubCommand = nh.advertise<mav_msgs::Actuators>("/commandData", 1);
    ros::Publisher  pubLogData = nh.advertise<quad_control::LogData>("/logData", 1);

    /*
     * Callback functions
     */
    void cbSensorRead(quad_control::UavStatePtr msg);

    void cbTrajectoryRead(quad_control::DesiredTrajectoryPtr msg);

    /*
     *  Program's functions
     */
    void updateUavState();
    void controlLoop();
    void sendCommand();
    void momentumBasedEstimator();
    void logDataPub();
};