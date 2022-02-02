#pragma once

#include <cmath>
#include <Eigen/Dense>

#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>

#include "../Utility/utility.hpp"

using Vector6d = Eigen::Matrix<double,6,1>;
using Vector3d = Eigen::Matrix<double,3,1>;
using Matrix3d = Eigen::Matrix<double,3,3>;
using Matrix6d = Eigen::Matrix<double,6,6>;

/*
 * This is a Drone Object.
 */
class Drone{

public:
    Drone(const double m, const double Ixx, const double Iyy, const double Izz);

    void updateState(const Vector6d pose, const Vector6d twist, const Vector6d wrench);

    void updateDesiredLinearState(const Vector3d pose_d, const Vector3d twist_d, const Vector3d wrench_d);

    void updateDesiredRoll(const double pose_d, const double twist_d, const double wrench_d);

    void updateDesiredPitch(const double pose_d, const double twist_d, const double wrench_d);
    
    void updateDesiredYaw(const double pose_d, const double twist_d, const double wrench_d);

    static Matrix3d getQ(const Vector3d eta);

    Vector3d p()                  { return getLinear(_Xi); };

    Vector3d pDot()               { return getLinear(_XiDot); };

    Vector3d pDotDot()            { return getLinear(_XiDotDot); };

    Vector3d eta()                { return getAngular(_Xi); };

    Vector3d etaDot()             { return getAngular(_XiDot); };

    Vector3d etaDotDot()          { return getAngular(_XiDotDot); };

    Vector3d p_d()                { return getLinear(_Xi_d); };

    Vector3d pDot_d()             { return getLinear(_XiDot_d); };

    Vector3d pDotDot_d()          { return getLinear(_XiDotDot_d); };

    Vector3d eta_d()              { return getAngular(_Xi_d); };

    Vector3d etaDot_d()           { return getAngular(_XiDot_d); };

    Vector3d etaDotDot_d()        { return getAngular(_XiDotDot_d); };

    Vector3d e_p()                { return getLinear(_Xi - _Xi_d); };

    Vector3d eDot_p()             { return getLinear(_XiDot - _XiDot_d); };

    Vector3d e_eta()              { return getAngular(_Xi - _Xi_d); };

    Vector3d eDot_eta()           { return getAngular(_XiDot - _XiDot_d); };

    double mass()                 { return _m; };

    Matrix3d m()                    { return _m * Matrix3d::Identity(); };

    Matrix3d I()                    { return _I; };

    Matrix3d Rb()                   { return _Rb; };

    Matrix3d Q()                    { return _Q; };

    Matrix3d M()                    { return ( Q().transpose() * I() * Q() ); };

    Matrix3d C()                    { return ( Q().transpose() * skew(Q() * etaDot()) * I() * Q() + Q().transpose() * I() * _Qdot); };

    Matrix3d Qtinv()                {return Q().transpose().inverse(); };

    Vector6d Xi()                   { return _Xi; };

    Vector6d XiDot()                { return _XiDot; };

    Vector6d XiDotDot()             { return _XiDotDot; };

    Matrix6d Mxi();
    
    Matrix6d Cxi();

    Vector6d Gxi();
    
    Eigen::Matrix<double,6,4> Dxi();

private:
    /**
     * @brief Mass of drone 
     * 
     */
    double _m;

    /**
     * @brief Inertia matrix of drone in drone NED frame
     * 
     */
    Matrix3d _I;

    /**
     * @brief Rotation matrix between the world frame and base frame
     * 
     */

    Matrix3d _Rb;

    /**
     * @brief Trasformation matrix from eta velocities in world frame to omega in world frame
     * 
     */
    Matrix3d _Q;

    /**
     * @brief Time derivative of Q
     * 
     */
    Matrix3d _Qdot;

    /**
     * @brief Pose of drone in world NED frame
     * 
     */
    Vector6d _Xi;

    /**
     * @brief Twist of drone in world NED frame
     * 
     */
    Vector6d _XiDot;

    /**
     * @brief Wrench of dtone in world NED frame
     * 
     */
    Vector6d _XiDotDot;

    /**
     * @brief Pose of drone in world NED frame
     * 
     */
    Vector6d _Xi_d;

    /**
     * @brief Twist of drone in world NED frame
     * 
     */
    Vector6d _XiDot_d;

    /**
     * @brief Wrench of dtone in world NED frame
     * 
     */
    Vector6d _XiDotDot_d;

    /**
     * @brief Sets the mass of drone
     * 
     * @param mass  mass of drone, it is mandatory that mass > 0 
     */
    void setMass(const double mass);

    /**
     * @brief Sets the inertia of drone respect the drone NED frame
     * 
     * @param Ixx Inertia of drone along x axis, it is mandatory that mass > 0
     * @param Iyy Inertia of drone along x axis, it is mandatory that mass > 0
     * @param Izz Inertia of drone along x axis, it is mandatory that mass > 0
     */
    void setInertia(const double Ixx, const double Iyy, const double Izz);

    /**
     * @brief Set the Rb object
     * 
     * @param eta 
     */
    void setRb(const Vector3d eta);

    /**
     * @brief Return time drivative of transformation matrix from omega to angular velocities
     *
     */
    Matrix3d getQdot(const Vector3d eta, const Vector3d eta_d);

    Vector3d getLinear(const Vector6d v);

    Vector3d getAngular(const Vector6d v);
};
