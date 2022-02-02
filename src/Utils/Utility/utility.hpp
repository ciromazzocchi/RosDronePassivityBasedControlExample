#pragma once

#include <cmath>
#include <Eigen/Dense>

#include <geometry_msgs/Twist.h>

using Vector2d = Eigen::Matrix<double,2,1>;
using Vector3d = Eigen::Matrix<double,3,1>;
using Vector4d = Eigen::Matrix<double,4,1>;
using Vector6d = Eigen::Matrix<double,6,1>;
using Matrix3d = Eigen::Matrix<double,3,3>;
using Matrix4d = Eigen::Matrix<double,4,4>;
using Matrix6d = Eigen::Matrix<double,6,6>;

const double GRAVITY        = 9.81;
const double RESOLUTION     = 0.001;
    
/**
 * @brief Return the skew symmetrix matrix
 * 
 * @param v Vector of parameter vx,vy,vz
 * @return Eigen::Matrix3d 
 */
Matrix3d skew(Vector3d v);

void vector2msg(const Vector3d linear, const Vector3d angular, geometry_msgs::Twist &msg);

Vector3d rounding(Vector3d v);
Vector6d rounding(Vector6d v);
Matrix3d rounding(Matrix3d v);
double rounding(double v);
void correctW(Vector4d &w2);

Matrix3d getQ(const Vector3d eta);
Matrix3d getQdot(const Vector3d eta, const Vector3d eta_d);