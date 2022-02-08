#include "./utility.hpp"
Matrix3d skew(Vector3d v)
{
    Matrix3d s;
    s <<    0,      -v[2],  v[1],
            v[2],   0,      -v[0],
            -v[1],  v[0],   0;
    return s;
}

Matrix3d getR(const Vector3d eta)
{
  Matrix3d R;
  double phi = eta[0]; double th = eta[1]; double psi = eta[2];
  R << cos(th)*cos(psi),
      sin(phi)*sin(th)*cos(psi)-cos(phi)*sin(psi),
      cos(phi)*sin(th)*cos(psi)+sin(phi)*sin(psi),
      cos(th)*sin(psi),
      sin(phi)*sin(th)*sin(psi)+cos(phi)*cos(psi),
      cos(phi)*sin(th)*sin(psi)-sin(phi)*cos(psi),
      -sin(th),
      sin(phi)*cos(th),
      cos(phi)*cos(th);

  return R;
}

Matrix3d getQ(const Vector3d eta) {
    double phi = eta(0);
    double theta = eta(1);

    Matrix3d Q;
    Q << 1, 0, -sin(theta),
         0, cos(phi), cos(theta)*sin(phi),
         0, -sin(phi), cos(theta)*cos(phi);
    return Q;
};

Matrix3d getQdot(const Vector3d eta, const Vector3d eta_d) {
    double phi = eta(0);
    double theta = eta(1);
    double phi_d = eta_d(0);
    double theta_d = eta_d(1);
    
    Matrix3d Q_dot;
    Q_dot << 0, 0, -cos(theta)*theta_d,
             0, -sin(phi)*phi_d, -sin(theta)*sin(phi)*theta_d+cos(theta)*cos(phi)*phi_d,
             0, -cos(phi)*phi_d, -sin(theta)*cos(phi)*theta_d-cos(theta)*sin(phi)*phi_d;

    return Q_dot;
};

Matrix3d M(const Vector3d eta, const Matrix3d I)
{
  return getQ(eta).transpose() * I * getQ(eta);
}

Matrix3d C(const Vector3d eta, const Vector3d eta_d, const Matrix3d I) {
  Eigen::Matrix3d Q = getQ(eta);
  return Q.transpose()*skew(Q*eta_d)*I*Q+Q.transpose()*I*getQdot(eta,eta_d);
}