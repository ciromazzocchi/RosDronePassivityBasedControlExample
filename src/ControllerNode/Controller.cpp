#include "./Controller.hpp"

Controller::Controller(){
    ROS_INFO_STREAM("Controller node - Parameters:");
    ROS_INFO_STREAM("Controller node - Frequency: " << this->rate);

    // UAV Initialization
    ROS_INFO_STREAM("Controller node - UAV mass: " << UAV.m());
    ROS_INFO_STREAM("Controller node - UAV Inertia:\n" << UAV.I());
    
    Vector6d initialPose;
    initialPose << Vector3d(0,0,-0.06), Vector3d::Zero(); 

    UAV.updateState(initialPose, Vector6d::Zero(), Vector6d::Zero());

    // Desired trajectory initialization
    UAV.updateDesiredLinearState(UAV.p(), UAV.pDot(), UAV. pDotDot());
    UAV.updateDesiredRoll(UAV.eta()(0), UAV.etaDot()(0), UAV.etaDotDot()(0));
    UAV.updateDesiredPitch(UAV.eta()(1), UAV.etaDot()(1), UAV.etaDotDot()(1));
    UAV.updateDesiredYaw(UAV.eta()(2), UAV.etaDot()(2), UAV.etaDotDot()(2));

    // Outer-loop parameter
    ROS_INFO_STREAM("Controller node - Kp matrix:\n" << Kp);
    ROS_INFO_STREAM("Controller node - Kd matrix:\n" << Kd);

    // Inner loop parameter
    ROS_INFO_STREAM("Controller node - v: " << this->v);
    ROS_INFO_STREAM("Controller node - K0 matrix:\n" << K0);
    ROS_INFO_STREAM("Controller node - D0 matrix:\n" << D0);

    // Initializes Filter
    eta_filter.initFilterStep( this->Ts, this->nh.param<double>("kf1",1), 
        this->nh.param<double>("kf2",1), Vector3d::Zero(),Vector3d::Zero());

    poseFilter.initFilterStep( this->Ts, this->nh.param<double>("kf1",1), 
        this->nh.param<double>("kf2",1), Vector6d::Zero(),Vector6d::Zero());
    
    double c0 = 4.6 / this->nh.param<double>("ta1_wrench_estimator", 0.001);
    wrenchEstimatorFilter.FilterInit(1, c0, 0, c0, this->Ts);   
}

void Controller::run()
{
    ROS_INFO_STREAM("Controller node - VTOL UAV Controller initialization started at " << ros::Time::now());

    ros::Duration(1).sleep();

    ros::spin();
}

/*
 *  Callback
 */
void Controller::cbSensorRead(quad_control::UavStatePtr msg)
{
    odomMsg = *msg;

    this->stamp = msg->header.stamp;

    updateUavState();

    //momentumBasedEstimator();

    controlLoop();
        
    sendCommand();
    
    logDataPub();
}


void Controller::cbTrajectoryRead(quad_control::DesiredTrajectoryPtr msg)
{
    UAV.updateDesiredLinearState(   Vector3d(msg->position.x, msg->position.y, msg->position.z),
                                    Vector3d(msg->twist.x, msg->twist.y, msg->twist.z),
                                    Vector3d(msg->wrench.x, msg->wrench.y, msg->wrench.z)
    );

    UAV.updateDesiredYaw(msg->position.yaw,msg->twist.yaw,msg->wrench.yaw);
}

/*
 * Block implementation
 */
void Controller::updateUavState() {
    Vector6d Pose,Twist,Wrench;

    Pose << odomMsg.pose.linear.x, odomMsg.pose.linear.y, odomMsg.pose.linear.z,
        odomMsg.pose.angular.x, odomMsg.pose.angular.y, odomMsg.pose.angular.z;

    poseFilter.filterStep(Pose);
    
    Pose << poseFilter.lastFiltered();
    Twist << poseFilter.lastFirst();
    Wrench << poseFilter.lastSecond();
    
    UAV.updateState(Pose,Twist,Wrench);
};

void Controller::controlLoop() {
    Vector3d fe = Vector3d(this->Fe(0), this->Fe(1), this->Fe(2));
    Vector3d taue = Vector3d(this->Fe(3), this->Fe(4),  this->Fe(5));
        
    /* Position Passive Loop*/
    Vector3d mu = UAV.pDotDot_d() - (Kp*UAV.e_p() + Kd*UAV.eDot_p())/UAV.mass();    // Desired acceleration

    Vector3d mu_hat = mu - (fe / UAV.mass());                           // Desired acceleration corrected
    mu_hat(2) = (mu_hat(2) >  0.9*GRAVITY) ?  0.9*GRAVITY : mu_hat(2);  // Avoids singularity
    mu_hat(2) = (mu_hat(2) < -0.9*GRAVITY) ? -0.9*GRAVITY : mu_hat(2);  // Avoids singularity
    mu_hat(2) = mu_hat(2) - GRAVITY; 

    /* Thrust references*/
    this->uT = UAV.mass() * mu_hat.norm();

    /* Attitude references */
    double psi_d = UAV.eta_d()(2);
    double psiDot_d = UAV.etaDot_d()(2);
    double psiDotDot_d = UAV.etaDotDot_d()(2);
    
    //Get desired yaw
    double phi_d = UAV.mass() * ( (mu_hat.y()*cos(psi_d) - mu_hat.x()*sin(psi_d)) / uT );
    if (phi_d < -0.9) phi_d = -0.9;     // Avoids overflow of asin argument
    if (phi_d >  0.9) phi_d =  0.9;     // Avoid underflow of asin argument
    phi_d = asin(phi_d);

    // Get desired pitch
    double theta_d = atan( (mu_hat.x()/mu_hat.z())*cos(psi_d) + (mu_hat.y()/mu_hat.z())*sin(psi_d) );

    // Get desired attitudes
    this->eta_filter.filterStep( Vector3d(phi_d, theta_d, psi_d) );
    Vector3d pose   = this->eta_filter.lastFiltered();
    Vector3d twist  = this->eta_filter.lastFirst();
    Vector3d wrench = this->eta_filter.lastSecond();

    UAV.updateDesiredRoll( pose(0), twist(0), wrench(0));
    UAV.updateDesiredPitch( pose(1), twist(1), wrench(1));
    
    /* Attitude Passive Control Loop */
        
    Vector3d etaDot_r    = UAV.etaDot_d()    - this->v * UAV.e_eta();
    Vector3d etaDotDot_r = UAV.etaDotDot_d() - this->v * UAV.eDot_eta();
    Vector3d v_eta       = UAV.eDot_eta()    + this->v * UAV.e_eta();
    
    this->taub = UAV.Qtinv() * ( UAV.M() * etaDotDot_r + UAV.C() * etaDot_r - taue - (D0 * v_eta) - (K0 * UAV.e_eta()) );
}

void Controller::sendCommand() {
    Vector4d ftau;
    ftau << this->uT, this->taub;
    
    commMsg.angular_velocities.resize(4);
    
    Matrix4d _G;
    double _l = 0.17; //meters
    double _c_T = 8.06428e-06;
    double _c_a = -0.016*_c_T;

    _G(0,0) = _c_T;     _G(0,1) = _c_T;    _G(0,2) = _c_T;    _G(0,3) = _c_T;
    _G(1,0) = 0;        _G(1,1) = _l*_c_T; _G(1,2) = 0;       _G(1,3) = -_l*_c_T;
    _G(2,0) = -_l*_c_T; _G(2,1) = 0;       _G(2,2) = _l*_c_T; _G(2,3) = 0;
    _G(3,0) = -_c_a;    _G(3,1) = _c_a;    _G(3,2) = -_c_a;   _G(3,3) = _c_a;

    Vector4d w2 = _G.inverse() * ftau;

    correctW(w2);
    
    commMsg.header.stamp = ros::Time::now();
    commMsg.angular_velocities[0] = sqrt(w2(3));
    commMsg.angular_velocities[1] = sqrt(w2(2));
    commMsg.angular_velocities[2] = sqrt(w2(1));
    commMsg.angular_velocities[3] = sqrt(w2(0));
    
    this->pubCommand.publish(commMsg);

}

void Controller::momentumBasedEstimator() {
    // Memorizes the wrench disturbance vector in k and k-i times
    Eigen::Matrix<double,4,1> U; U << this->uT, this->taub;
    Vector6d Fe = UAV.Cxi().transpose() * UAV.XiDot() + UAV.Dxi()*U - UAV.Gxi();
    this->Fe = this->wrenchEstimatorFilter.getFilteredValue(Fe);
}

void Controller::logDataPub() {
    
    quad_control::LogData pub_msg;
    Vector6d temp;

    pub_msg.header.stamp = ros::Time::now();
    pub_msg.header.seq = odomMsg.header.seq;

    // Sensor's data
    vector2msg(UAV.p(), UAV.eta(), pub_msg.pose);
    vector2msg(UAV.pDot(), UAV.etaDot(), pub_msg.twist);
    vector2msg(UAV.pDotDot(), UAV.etaDotDot(), pub_msg.wrench);

    // Desired's data
    vector2msg(UAV.p_d(), UAV.eta_d(), pub_msg.desiredPose);
    vector2msg(UAV.pDot_d(), UAV.etaDot_d(), pub_msg.desiredTwist);
    vector2msg(UAV.pDotDot_d(), UAV.etaDotDot_d(), pub_msg.desiredWrench);

    // Error's data
    vector2msg(UAV.e_p(), UAV.e_eta() ,pub_msg.error_pose);
    vector2msg(UAV.eDot_p(), UAV.eDot_eta(), pub_msg.error_twist);
    vector2msg(UAV.pDotDot() - UAV.pDotDot_d(), UAV.etaDotDot() - UAV.etaDotDot_d(),
        pub_msg.error_wrench);

    // Esimated wrench's data
    vector2msg(Vector3d(this->Fe(0),this->Fe(1), this->Fe(2)), 
        Vector3d(this->Fe(3), this->Fe(4), this->Fe(5)), pub_msg.estimatedWrench);

    // Propeller's data
    vector2msg(Vector3d(0,0, this->uT), this->taub, pub_msg.propellersInput);

    this->pubLogData.publish(pub_msg);
}