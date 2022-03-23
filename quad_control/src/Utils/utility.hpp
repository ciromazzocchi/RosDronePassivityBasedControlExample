#pragma once

#include <cmath>
#include <Eigen/Dense>

using Vector3d = Eigen::Matrix<double,3,1>;
using Matrix3d = Eigen::Matrix<double,3,3>;
    
/**
 * @brief Return the skew symmetrix matrix
 * 
 * @param v Vector of parameter vx,vy,vz
 * @return Eigen::Matrix3d 
 */
Matrix3d skew(Vector3d v);
Vector3d getEta(Eigen::Quaterniond q);
Matrix3d getR(const Vector3d eta);
Matrix3d getQ(const Vector3d eta);
Matrix3d getQdot(const Vector3d eta, const Vector3d eta_d);
Matrix3d getM(const Vector3d eta, const Matrix3d I);
Matrix3d getC(const Vector3d eta, const Vector3d eta_d, const Matrix3d I);