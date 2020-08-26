#ifndef _RL_ALTITUDE_CONTROL_H_
#define _RL_ALTITUDE_CONTROL_H_
#include <mavros_msgs/AttitudeTarget.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <dynamic_reconfigure/server.h>
// #include <drone_new/dynamic_paramConfig.h>
#include <sensor_msgs/Imu.h>

#define minimum_snap_Row 9
#define minimum_snap_Col 24
#define R_circle 2
#define mass 1.5
#define g 9.8
#define attctrl_tau_ 0.1
double traj_T;

char finish_flip = 0;
double traj_omega_;
double KR1,KR2,KR3,KOmega1,KOmega2,KOmega3;
double k_pos1,k_pos2,k_pos3,k_vel1,k_vel2,k_vel3;
Eigen::Vector3d traj_radial_(0.75, 0, 0);
Eigen::Vector3d traj_axis_(0, 0, 1);
Eigen::Vector3d traj_origin_(0, 0, 1);
Eigen::Matrix<double, minimum_snap_Row, minimum_snap_Col> _polyCoeff;
Eigen::Matrix<double, minimum_snap_Row, 1> _polyTime;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_pos_position;
Eigen::Vector3d pos_current;
geometry_msgs::TwistStamped local_vel_current;
geometry_msgs::TwistStamped body_vel_current;
Eigen::Vector3d ver_current,AngleRate_current;
mavros_msgs::AttitudeTarget local_attitude_target;
sensor_msgs::Imu imu_data;

class rl_altitude_control
{
public:
    double time;
    Eigen::Vector3d getPosition(double time, std::string traj_type);
    Eigen::Vector3d getVelocity(double time, std::string traj_type);
    Eigen::Vector3d getAcceleration(double time, std::string traj_type);
    Eigen::Vector3d getjerk(double time, std::string traj_type);
    Eigen::Vector3d getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t);
    Eigen::Vector3d getVelPoly(Eigen::MatrixXd polyCoeff, int k, double t);
    Eigen::Vector3d getAccPoly(Eigen::MatrixXd polyCoeff, int k, double t);
    void visWayPointTraj(const Eigen::Matrix<double, minimum_snap_Row, minimum_snap_Col> &polyCoeff,
                         const Eigen::Matrix<double, minimum_snap_Row, 1> &timer, double time,
                         Eigen::Vector3d &Trajectory_pos, Eigen::Vector3d &Trajectory_vel,
                         Eigen::Vector3d &Trajectory_acc);
};

inline Eigen::Vector3d ToEigen(const geometry_msgs::Point &point)
{
    Eigen::Vector3d tmp;
    tmp << point.x, point.y, point.z;
    return tmp;
}
inline Eigen::Vector3d ToEigen(const geometry_msgs::Vector3 &ev)
{
    Eigen::Vector3d tmp;
    tmp << ev.x, ev.y, ev.z;
    return tmp;
}

Eigen::Vector3d attcontroller(const Eigen::Vector4d &att_ref, const Eigen::Vector4d &curr_att)
{
    Eigen::Vector4d qe, q_inv, inverse;
    Eigen::Vector3d ratecmd;

    inverse << 1.0, -1.0, -1.0, -1.0;
    q_inv = inverse.asDiagonal() * curr_att; //四元数 求逆，虚部相反数
    qe = quatMultiplication(q_inv, att_ref);
    ratecmd(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
    ratecmd(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
    ratecmd(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
    return ratecmd;
}

Eigen::Vector3d matrix_hat_inv(const Eigen::Matrix3d &m) {
  Eigen::Vector3d v;
  //TODO: Sanity checks if m is skew symmetric
  v << m(7), m(2), m(3);
  return v;
}

Eigen::Matrix3d matrix_hat(const Eigen::Vector3d &v){
    Eigen::Matrix3d m;
    m << 0,     -v(2),   v(1),
        v(2),       0,  -v(0),
        -v(1),   v(0),      0;
    return m;
}

Eigen::Vector3d geometric_attcontroller(const Eigen::Vector4d &ref_att, Eigen::Vector4d &curr_att){
  // Geometric attitude controller
  // Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.  
  // The original paper inputs moment commands, but for offboard control angular rate commands are sent

  Eigen::Vector3d ratecmd;
  Eigen::Matrix3d rotmat; //Rotation matrix of current atttitude
  Eigen::Matrix3d rotmat_d; //Rotation matrix of desired attitude
  Eigen::Vector3d zb;
  Eigen::Vector3d error_att;

  rotmat = quat2RotMatrix(curr_att);
  rotmat_d = quat2RotMatrix(ref_att);

  error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat);
  ratecmd = (2.0 / attctrl_tau_) * error_att;
  return ratecmd;
}

#define FLIP_N 1
Eigen::Vector4d attitude_flip(const Eigen::Vector4d &ref_att, Eigen::Vector4d &curr_att, double t){

  Eigen::Vector4d ratecmd;
  Eigen::Matrix3d rotmat; //Rotation matrix of current atttitude
  Eigen::Matrix3d rotmat_d; //Rotation matrix of desired attitude
  Eigen::Vector3d zb;
  Eigen::Vector3d error_att;
  Eigen::Vector3d er(1,0,0);

  double u1 = 0.9;
  double u5 = 0.9;
  double u2 = 0.45;
  double beta_max = 2.8*4/0.468;
  double beta_min = 0.08*4/0.468;
  double alpha = 2*0.0023/(0.468*0.17*0.17);
  double ddot_theta2 = (beta_max-beta_min)/(2*alpha*0.17);
  double dot_theta_max = 220;    //1200
  double t3 = 2*M_PI*FLIP_N/dot_theta_max - dot_theta_max/ddot_theta2;

  double dd_theta1 = (beta_max - 0.9*beta_max)/(alpha*0.17);
  double t1 = 0.2;                                              //This is desigened
  double t2 = (dot_theta_max-dd_theta1*t1)/ddot_theta2;

  rotmat = quat2RotMatrix(curr_att);
  static double sum=0;
//   std::cout<<t2<<"\t"<<t3<<"\t"<<t4<<"\t"<<t5;
//   rotmat_d = quat2RotMatrix(ref_att);
  t2 = 0.2;
  t3 = 0.1;
  double t4 = t2;
  double t5 = t1;

    // std::cout<<er*er.transpose()<<std::endl;
  if(t<t1){
        // rotmat_d = Eigen::Matrix3d::Identity()+ std::sin(4*M_PI*t)*matrix_hat(er) 
        //         + (1-std::cos(4*M_PI*t))*(er*er.transpose() - Eigen::Matrix3d::Identity());
        // error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat);

        ratecmd = Eigen::Vector4d::Zero();
        ratecmd(3) = u1;
  }
  else if(t<(t1+t2))
  {
      ratecmd = Eigen::Vector4d::Zero();
      sum += 24/5;
      ratecmd(1) =  sum;
      ratecmd(3) = u2;
  }
  else if(t<(t1+t2+t3))
  {
      ratecmd = Eigen::Vector4d::Zero();
      ratecmd(1) = dot_theta_max;
      ratecmd(3) = 0.08;
  }
  else if(t<(t1+t2+t3+t4))
  {
      ratecmd = Eigen::Vector4d::Zero();
      sum -= 24/5;
      ratecmd(1) = sum;
      ratecmd(3) = u2;
  }
  else if(t<(t1+t2+t3+t4+t5)){
        // Eigen::Vector4d ref_att(1,0,0,0);
        //error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat);
        ratecmd = Eigen::Vector4d::Zero();
        ratecmd(1) = 0;
        ratecmd(3) = 0.9;
  }
  else{
      ratecmd = Eigen::Vector4d::Zero();
      ratecmd(1) = 0;
      ratecmd(3) = 0.6;
  }
  std::cout<<ratecmd.transpose()<<"\t"<<std::endl;

  return ratecmd;
}

Eigen::Vector3d geometry_2011_acc(const Eigen::Vector4d &ref_att, Eigen::Vector4d &curr_att){
  // Geometric attitude controller
  // Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.  
  // The original paper inputs moment commands, but for offboard control angular rate commands are sent

  Eigen::Vector3d ratecmd;
  Eigen::Matrix3d rotmat; //Rotation matrix of current atttitude
  Eigen::Matrix3d rotmat_d; //Rotation matrix of desired attitude
  Eigen::Vector3d zb;
  Eigen::Vector3d error_att;

  rotmat = quat2RotMatrix(curr_att);
  rotmat_d = quat2RotMatrix(ref_att);
  Eigen::Matrix3d matrixQ = rotmat_d.transpose()*rotmat;

  error_att = 0.5 / sqrt( 1 + matrixQ.trace()) * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat);
  ratecmd = (2.0 / attctrl_tau_) * error_att;
  return ratecmd;
}

Eigen::Vector3d rate_controller(const Eigen::Vector4d &ref_att, 
                                Eigen::Vector4d &curr_att,double psi,
                                const Eigen::Vector3d dot_a_des,const Eigen::Vector3d a_des,
                                const Eigen::Vector3d proj_xb_des){
    // Geometric attitude controller
    // Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.  
    // The original paper inputs moment commands, but for offboard control angular rate commands are sent
    // In this mode ,the inner rate_pid in px4 must be put the kp =0.5 and ki = kd = 0 for the truly use

    Eigen::Vector3d ratecmd;
    Eigen::Matrix3d rotmat; //Rotation matrix of current atttitude
    Eigen::Matrix3d rotmat_d; //Rotation matrix of desired attitude
    Eigen::Vector3d zb;
    Eigen::Vector3d error_att;
    Eigen::Vector3d Error_AngleVelocity;

    rotmat = quat2RotMatrix(curr_att);
    rotmat_d = quat2RotMatrix(ref_att);

    error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat);
    double d_psi = 0; //first, suppose the d_psi = 0;

    Eigen::Vector3d dot_proj_xb_des(-d_psi*sin(psi),d_psi*cos(psi),0);
    Eigen::Vector3d dot_f_norm = dot_a_des / a_des.norm();
    Eigen::Vector3d dot_b3_LefTerm = rotmat_d.col(2).cross(dot_f_norm);
    Eigen::Vector3d dot_b3 = dot_b3_LefTerm.cross(rotmat_d.col(2));
    // std::cout<<dot_a_des.transpose()<<"\t"<<dot_b3_RightTerm.transpose()<<std::endl;

    Eigen::Vector3d dotb2desCROSSb3 = dot_proj_xb_des.cross(rotmat_d.col(2)) + proj_xb_des.cross(dot_b3);
    Eigen::Vector3d b2desCROSSb3 =  proj_xb_des.cross(rotmat_d.col(2));
    Eigen::Vector3d dot_b1_MiddleTerm_norm = dotb2desCROSSb3 / b2desCROSSb3.norm();
    Eigen::Vector3d dot_b1_LeftTerm = rotmat_d.col(0).cross(dot_b1_MiddleTerm_norm);
    Eigen::Vector3d dot_b1 = dot_b1_LeftTerm.cross(rotmat_d.col(0));

    Eigen::Vector3d dot_b2 = dot_b3.cross(rotmat_d.col(0)) + rotmat_d.col(2).cross(dot_b1);

    Eigen::Matrix3d dot_R_des;
    dot_R_des<< dot_b1(0), dot_b2(0), dot_b3(0),
            dot_b1(1), dot_b2(1), dot_b3(1),
            dot_b1(2), dot_b2(2), dot_b3(2);
  
    Eigen::Vector3d AngleVelocityRef = matrix_hat_inv(rotmat_d.transpose()*dot_R_des);

    Error_AngleVelocity = AngleRate_current - rotmat.transpose()*rotmat_d*AngleVelocityRef;
    // std::cout<<"Mavros:"<<Error_AngleVelocity.transpose()<<std::endl;

    Eigen::Vector3d KR,KOmega;
    KR<<KR1,KR2,KR3;
    KOmega<<KOmega1,KOmega2,KOmega3;
    const Eigen::Vector3d J(0.003544,0.003544,0.006935);
    Eigen::Vector3d dotOmegaRef;
    static Eigen::Vector3d OmegaRef_last(0,0,0);
    double dt = 1.0/250.0;
    dotOmegaRef = (AngleVelocityRef - OmegaRef_last)/dt;
    if(dotOmegaRef.norm()>10)
        dotOmegaRef = (10/dotOmegaRef.norm()) * dotOmegaRef;

    ratecmd = KR.asDiagonal()*error_att + KOmega.asDiagonal()*Error_AngleVelocity;
            // - AngleRate_current.cross(J.asDiagonal()*AngleRate_current)
                // + J.asDiagonal()*(matrix_hat(AngleRate_current)*rotmat.transpose()*rotmat_d*AngleVelocityRef - rotmat.transpose()*rotmat_d*dotOmegaRef);
    OmegaRef_last = AngleVelocityRef;
    // std::cout<<ratecmd.transpose()<<std::endl;
    
    return ratecmd;
}

void Circle_trajectory(const sensor_msgs::Imu &imu_data,const geometry_msgs::PoseStamped &pos, const ros::Time &t1, 
                            std::string traj_type, std::string controller)
{
    Eigen::Vector3d Trajectory_pos, Trajectory_vel, Trajectory_acc, Trajectory_jerk;
    ros::Duration t = ros::Time::now() - t1;
    rl_altitude_control rl;
    rl.time = t.toSec();
    traj_omega_ = 2 * M_PI / traj_T;
    Trajectory_pos = rl.getPosition(rl.time, traj_type);
    Trajectory_vel = rl.getVelocity(rl.time, traj_type);
    Trajectory_acc = rl.getAcceleration(rl.time, traj_type);
    Trajectory_jerk = rl.getjerk(rl.time, traj_type);
    Eigen::Vector3d pos_err = pos_current - Trajectory_pos;
    Eigen::Vector3d vel_err = ver_current - Trajectory_vel;

    // Adopt the feedforward style to calculate the error
    double dt = 1.0/250.0;
    static Eigen::Vector3d ver_current_last(0,0,0);
    Eigen::Vector3d acc_current = (ver_current - ver_current_last)/dt;
    Eigen::Vector3d acc_err = acc_current - Trajectory_acc;

    Eigen::Vector3d k_pos, k_vel;
    k_pos << k_pos1,k_pos2,k_pos3;
    k_vel << k_vel1,k_vel2,k_vel3;

    Eigen::Vector4d quat_current;
    quat_current << imu_data.orientation.w, imu_data.orientation.x,
        imu_data.orientation.y, imu_data.orientation.z;
    Eigen::Matrix3d rotation_current;
    rotation_current = quat2RotMatrix(quat_current);
    double thrust;

    static double psi = M_PI;
    // const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel; //Rotor drag
    psi = M_PI + traj_omega_ * (rl.time - (int)(rl.time/traj_T)*traj_T);  // has been tested for circle flip
    // ROS_INFO_STREAM_THROTTLE(0.01,psi);
    Eigen::Vector3d a_fb = k_pos.asDiagonal() * pos_err + k_vel.asDiagonal() * vel_err;

    if (a_fb.norm() > 8)
        a_fb = (8 / a_fb.norm()) * a_fb;

    const Eigen::Vector3d a_des = a_fb + g * Eigen::Vector3d(0, 0, 1) + Trajectory_acc;
    const Eigen::Vector3d dot_a_des = k_pos.asDiagonal()* vel_err + k_vel.asDiagonal() * acc_err + Trajectory_jerk;
    thrust = 0.05 * a_des.dot(rotation_current * Eigen::Vector3d(0, 0, 1));

    Eigen::Vector3d proj_xb_des(cos(psi), sin(psi), 0);
    Eigen::Vector3d zb_des = a_des / a_des.norm();
    Eigen::Vector3d yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des).norm());
    Eigen::Vector3d xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();
    Eigen::Matrix3d R_des;
    R_des << xb_des(0), yb_des(0), zb_des(0),
        xb_des(1), yb_des(1), zb_des(1),
        xb_des(2), yb_des(2), zb_des(2);


    Eigen::Vector4d quat_des = rot2Quaternion(R_des);
    Eigen::Vector4d quat_curr(quat_current);
    Eigen::Vector3d ratecmd;
    Eigen::Vector4d ratecmd_all;
    if(controller == "attitude_eth"){
        ratecmd = attcontroller(quat_des, quat_curr);
    }
    else if(controller == "attitude_lee"){
        ratecmd = geometric_attcontroller(quat_des,quat_curr);
    }
    else if(controller == "rate_controller"){
        ratecmd = rate_controller(quat_des,quat_curr,psi,dot_a_des,a_des,proj_xb_des);
    }
    else if(controller == "geometry_2011_acc"){
        ratecmd = geometry_2011_acc(quat_des,quat_curr);
    }
    else if(controller == "attitude_flip"){
        ratecmd_all = attitude_flip(quat_des,quat_curr,rl.time);
    }
    local_attitude_target.body_rate.x = ratecmd(0);
    local_attitude_target.body_rate.y = ratecmd(1);
    local_attitude_target.body_rate.z = ratecmd(2);
    
    if(controller == "attitude_flip"){
        local_attitude_target.body_rate.x = ratecmd_all(0);
        local_attitude_target.body_rate.y = ratecmd_all(1);
        local_attitude_target.body_rate.z = ratecmd_all(2);
        local_attitude_target.thrust = std::max(0.0, std::min(1.0, ratecmd_all(3)));
        if(ratecmd_all(3)==0.6)
        {
            finish_flip = 1;
        }
    }    
    else{
        local_attitude_target.thrust = std::max(0.0, std::min(1.0, thrust + 0.1));
    }
        
    local_attitude_target.type_mask = 0b10111000;

    ver_current_last = ver_current;  //pass the linear velocity to the last one for calculate the linear acc 
}

Eigen::Vector3d rl_altitude_control::getPosition(double time, std::string traj_type)
{
    Eigen::Vector3d position;
    double theta;

    theta = traj_omega_ * time;
    if (traj_type == "Lemn"){

        position = std::cos(theta) * traj_radial_ + std::sin(theta) * std::cos(theta) * traj_axis_.cross(traj_radial_) + (1 - std::cos(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_ + traj_origin_;
    }
    else
    {
        position = std::cos(theta) * traj_radial_ + std::sin(theta) * traj_axis_.cross(traj_radial_) + (1 - std::cos(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_ + traj_origin_;
    }
    return position;
}

Eigen::Vector3d rl_altitude_control::getVelocity(double time, std::string traj_type)
{
    Eigen::Vector3d velocity;
    velocity = traj_omega_ * traj_axis_.cross(getPosition(time, traj_type));
    return velocity;
}

Eigen::Vector3d rl_altitude_control::getAcceleration(double time, std::string traj_type)
{
    Eigen::Vector3d acceleration;
    acceleration = traj_omega_ * traj_axis_.cross(getVelocity(time, traj_type));
    return acceleration;
} // this method only useful for circle trajectory

Eigen::Vector3d rl_altitude_control::getjerk(double time, std::string traj_type)
{
    Eigen::Vector3d jerk;
    jerk = traj_omega_ * traj_axis_.cross(getAcceleration(time, traj_type));
    return jerk;
} 

Eigen::Vector3d rl_altitude_control::getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t)
{
    Eigen::Vector3d ret;
    uint8_t _poly_num1D = 8;
    for (int dim = 0; dim < 3; dim++)
    {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
        Eigen::VectorXd time = Eigen::VectorXd::Zero(_poly_num1D);

        for (int j = 0; j < _poly_num1D; j++)
        {
            if (j == 0)
                time(j) = 1.0;
            else
                time(j) = pow(t, j);
        }
        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }
    return ret;
}

Eigen::Vector3d rl_altitude_control::getVelPoly(Eigen::MatrixXd polyCoeff, int k, double t)
{
    Eigen::Vector3d ret;
    uint8_t _poly_num1D = 8;
    for (int dim = 0; dim < 3; dim++)
    {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
        Eigen::VectorXd time = Eigen::VectorXd::Zero(_poly_num1D);

        for (int j = 0; j < _poly_num1D; j++)
        {
            if (j == 0)
                time(j) = 0.0;
            else if (j == 1)
                time(j) = 1.0;
            else
                time(j) = j * pow(t, j - 1);
        }
        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }
    return ret;
}

Eigen::Vector3d rl_altitude_control::getAccPoly(Eigen::MatrixXd polyCoeff, int k, double t)
{
    Eigen::Vector3d ret;
    uint8_t _poly_num1D = 8;
    for (int dim = 0; dim < 3; dim++)
    {
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
        Eigen::VectorXd time = Eigen::VectorXd::Zero(_poly_num1D);

        for (int j = 0; j < _poly_num1D; j++)
        {
            if (j == 0 || j == 1)
                time(j) = 0.0;
            else if (j == 2)
                time(j) = 2.0;
            else
                time(j) = j * (j - 1) * pow(t, j - 2);
        }
        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }
    return ret;
}

void rl_altitude_control::visWayPointTraj(const Eigen::Matrix<double, minimum_snap_Row, minimum_snap_Col> &polyCoeff,
                                          const Eigen::Matrix<double, minimum_snap_Row, 1> &timer, double time,
                                          Eigen::Vector3d &Trajectory_pos, Eigen::Vector3d &Trajectory_vel,
                                          Eigen::Vector3d &Trajectory_acc)
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Matrix<double, minimum_snap_Row, 1> time_cut;
    rl_altitude_control rl;
    for (uint8_t i = 0; i < timer.size(); i++)
    {
        if (i == 0)
        {
            time_cut(i) = 0;
        }
        else
        {
            time_cut(i) = time_cut(i - 1) + timer(i);
        }
    }
    // std::cout<<time_cut<<std::endl;
    if (time > time_cut(minimum_snap_Row - 1))
    {
        Trajectory_pos = Eigen::Vector3d(0, 0, 2);
        Trajectory_vel = Eigen::Vector3d::Zero();
        Trajectory_acc = Eigen::Vector3d::Zero();
    }
    else
    {
        for (uint8_t i = 0; i < minimum_snap_Row - 1; i++)
        {
            if (time >= time_cut(i) && time < time_cut(i + 1))
            {
                Trajectory_pos = getPosPoly(polyCoeff, i, time - time_cut(i));
                Trajectory_vel = getVelPoly(polyCoeff, i, time - time_cut(i));
                Trajectory_acc = getAccPoly(polyCoeff, i, time - time_cut(i));
            }
        }
    }

    // std::cout<<Trajectory_pos.transpose()<<std::endl;
}

#endif