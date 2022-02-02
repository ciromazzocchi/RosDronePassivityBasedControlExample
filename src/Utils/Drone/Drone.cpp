#include "./Drone.hpp"

/*
 *  Public function
 */

Drone::Drone(const double m, const double Ixx, const double Iyy, const double Izz){
    this->setMass(m);
    this->setInertia(Ixx,Iyy,Izz);
};

void Drone::updateState(const Vector6d pose, const Vector6d twist, const Vector6d wrench){
    this->_Xi = pose;
    this->_XiDot = twist;
    this->_XiDotDot = wrench;

    this->setRb(this->eta());

    _Q    = this->getQ(eta());

    _Qdot = this->getQdot(eta(),etaDot());
};

void Drone::updateDesiredLinearState(const Vector3d pose_d, const Vector3d twist_d, const Vector3d wrench_d){
    for(int i=0; i < 3; i++) {
        this->_Xi_d(i) = pose_d(i);
        this->_XiDot_d(i) = twist_d(i);
        this->_XiDotDot_d(i) = wrench_d(i);
    }
};

void Drone::updateDesiredRoll(const double pose_d, const double twist_d, const double wrench_d) {
    _Xi_d(3)        = pose_d;
    _XiDot_d(3)     = twist_d;
    _XiDotDot_d(3)  = wrench_d;
};

void Drone::updateDesiredPitch(const double pose_d, const double twist_d, const double wrench_d) {
    _Xi_d(4)        = pose_d;
    _XiDot_d(4)     = twist_d;
    _XiDotDot_d(4)  = wrench_d;
};
    
void Drone::updateDesiredYaw(const double pose_d, const double twist_d, const double wrench_d) {
    _Xi_d(5)        = pose_d;
    _XiDot_d(5)     = twist_d;
    _XiDotDot_d(5)  = wrench_d;
};

Matrix3d Drone::getQ(const Vector3d eta) {
    double phi = eta(0);
    double theta = eta(1);

    Matrix3d Q;
    Q << 1, 0, -sin(theta),
         0, cos(phi), cos(theta)*sin(phi),
         0, -sin(phi), cos(theta)*cos(phi);
    return Q;
};

Matrix3d Drone::getQdot(const Vector3d eta, const Vector3d eta_d) {
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

/* 
 *  Private function
 */
void Drone::setMass(const double mass){
    if (mass <= 0) {
        throw std::runtime_error("Mass must be major than 0");
    }
    this->_m = mass;
}

void Drone::setInertia(const double Ixx, const double Iyy, const double Izz) {
    if (Ixx <= 0 || Iyy <= 0 || Izz <= 0) {
        throw std::runtime_error("Inertias must be major than 0");
    }
    this->_I = Eigen::Vector3d(Ixx,Iyy,Izz).asDiagonal();
}

Matrix6d Drone::Mxi() {
    Matrix6d Mxi;
    Mxi  << m(), Matrix3d::Zero(), Matrix3d::Zero(), M();
    return Mxi;
}

Matrix6d Drone::Cxi() {
    Matrix6d Cxi = Matrix6d::Zero();
    for(int i = 0; i < 3; i++)
        for(int j=0; j<3; j++)
            Cxi(i+3,j+3) = C()(i,j);
    return Cxi;
}

Vector6d Drone::Gxi() {
    Vector6d Gxi;
    Gxi << 0,0, - GRAVITY * mass(), 0, 0, 0;
    return Gxi;
}

Eigen::Matrix<double,6,4> Drone::Dxi() {
    Eigen::Matrix<double,6,4> Dxi;
    Matrix3d Rb = this->Rb();
    Matrix3d Qt = Q().transpose();

    Dxi <<  -Rb(0,2), 0, 0, 0,
            -Rb(1,2), 0, 0, 0,
            -Rb(2,2), 0, 0, 0,
            0, Qt(0,0), Qt(0,1), Qt(0,2),
            0, Qt(1,0), Qt(1,1), Qt(1,2),
            0, Qt(2,0), Qt(2,1), Qt(2,2);
    return Dxi;
}

void Drone::setRb(const Vector3d eta)
{
    tf::Quaternion q;
    q.setRPY(eta(0),eta(1),eta(2));
    q.normalize();
    tf::Matrix3x3 tfRb;
    tfRb.getRotation(q);
    tf::matrixTFToEigen(tfRb, _Rb);
}

Vector3d Drone::getLinear(const Vector6d v) {
    Vector3d v2;
    for(int i=0; i < 3; i++)
        v2(i) = v(i);
    return v2;
}

Vector3d Drone::getAngular(const Vector6d v) {
    Vector3d v2;
    for(int i=0; i < 3; i++)
        v2(i) = v(3+i);
    return v2;
}