#include "./utility.hpp"
Matrix3d skew(Vector3d v)
{
    Matrix3d s;
    s <<    0,      -v[2],  v[1],
            v[2],   0,      -v[0],
            -v[1],  v[0],   0;
    return s;
}

void vector2msg(const Vector3d linear, const Vector3d angular, geometry_msgs::Twist &msg)
{
    msg.linear.x = linear(0);
    msg.linear.y = linear(1);
    msg.linear.z = linear(2);

    msg.angular.x = angular(0);
    msg.angular.y = angular(1);
    msg.angular.z = angular(2);
}

Vector3d rounding(Vector3d v) {
    for(int i=0; i<v.rows(); i++)
        v(i) = RESOLUTION * round(v(i) / RESOLUTION );
    return v;
}

Vector6d rounding(Vector6d v) {
    for(int i=0; i<v.rows(); i++)
        v(i) = RESOLUTION * round(v(i) / RESOLUTION );
    return v;
}

double rounding(double v) {
    return RESOLUTION * round(v / RESOLUTION );
}

Matrix3d rounding(Matrix3d v) {
    for(int i=0; i<v.rows(); i++)
        for(int j=0; j<v.cols(); j++)
            v(i,j) = RESOLUTION * round(v(i,j) / RESOLUTION );
    return v;
}

void correctW(Vector4d &w2) {
    if(w2(0)<0 && w2(2)<0) {
      w2(0)=0;
      w2(2)=0;
    } else {
      if (w2(0) < 0) {
        w2(2) += w2(0);
        w2(0) = 0;
      }
      if (w2(2) < 0) {
        w2(0) += w2(2);
        w2(2) = 0;
      }
    }

    if(w2(1)<0 && w2(3)<0) {
      w2(1)=0;
      w2(3)=0;
    } else {
      if (w2(1) < 0) {
        w2(3) += w2(1);
        w2(1) = 0;
      }
      if (w2(3) < 0) {
        w2(1) += w2(3);
        w2(3) = 0;
      }
    }

    for (int i=0; i<4; i++) {
      if(w2(i)<0) w2(i)=0;
      //ROS_INFO("w(%d): %f",i,w2(i));
    }
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