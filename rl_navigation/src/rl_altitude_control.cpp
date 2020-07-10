#include <mavros_msgs/AttitudeTarget.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <math.h>

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

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

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped local_pos_position;
Eigen::Vector3d pos_current;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos_position = *msg;
    pos_current = ToEigen(local_pos_position.pose.position);
}

geometry_msgs::TwistStamped local_vel_current;
Eigen::Vector3d ver_current;
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_vel_current = *msg;
    ver_current = ToEigen(local_vel_current.twist.linear);
}

#define R_circle 2
#define mass 1.5
#define g 9.8
mavros_msgs::AttitudeTarget local_attitude_target;
void quat2eular_show(const Eigen::Quaterniond &quatf)
{
    struct Quaternion quat_show;
    quat_show.w = quatf.w();
    quat_show.x = quatf.x();
    quat_show.y = quatf.y();
    quat_show.z = quatf.z();
    struct EulerAngles euler_show = ToEulerAngles(quat_show);
    // std::cout<<180/M_PI*euler_show.roll<<"\t"<<180/M_PI*euler_show.pitch<<"\t"<<180/M_PI*euler_show.yaw<<std::endl;
}

Eigen::Vector4f quatMultiplication(const Eigen::Vector4f &q, const Eigen::Vector4f &p) {
   Eigen::Vector4f quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3),
          p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
          p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1),
          p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}
#define attctrl_tau_ 0.1

Eigen::Vector3d attcontroller(const Eigen::Vector4f &att_ref, const Eigen::Vector4f &curr_att)
{
    Eigen::Vector4f  qe, q_inv, inverse;
    Eigen::Vector3d ratecmd;
    
    inverse<< 1.0, -1.0, -1.0, -1.0;
    q_inv = inverse.asDiagonal() * curr_att;   //四元数 求逆，虚部相反数
    qe = quatMultiplication(q_inv,att_ref);
    ratecmd(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
    ratecmd(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
    ratecmd(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
    return ratecmd;
}

Eigen::Vector4f rot2Quaternion(const Eigen::Matrix3d &R){
  Eigen::Vector4f quat;
  double tr = R.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}

Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4f &q){
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3),
    2 * q(1) * q(2) - 2 * q(0) * q(3),
    2 * q(0) * q(2) + 2 * q(1) * q(3),

    2 * q(0) * q(3) + 2 * q(1) * q(2),
    q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
    2 * q(2) * q(3) - 2 * q(0) * q(1),

    2 * q(1) * q(3) - 2 * q(0) * q(2),
    2 * q(0) * q(1) + 2 * q(2) * q(3),
    q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}

double traj_omega_ = 3;
Eigen::Vector3d traj_radial_(1,0,0);
Eigen::Vector3d traj_axis_(0,0,1);
Eigen::Vector3d traj_origin_(0,0,2);
Eigen::Vector3d getPosition(double time){
    Eigen::Vector3d position;
    double theta;

    theta = traj_omega_* time;
    position = std::cos(theta) * traj_radial_
                + std::sin(theta) * traj_axis_.cross(traj_radial_)
                + (1 - std::cos(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_
                + traj_origin_;
    return position;
}
Eigen::Vector3d getVelocity(double time){
    Eigen::Vector3d velocity;
    velocity = traj_omega_ * traj_axis_.cross(getPosition(time));
    return velocity;
}
Eigen::Vector3d getAcceleration(double time){
    Eigen::Vector3d acceleration;
    acceleration = traj_omega_ * traj_axis_.cross(getVelocity(time));
    return acceleration;
}                                               // this method only useful for circle trajectory

void Circle_trajectory(const geometry_msgs::PoseStamped &pos, const ros::Time &t1)
{
    Eigen::Vector3d Trajectory_pos,Trajectory_vel,Trajectory_acc;
    ros::Duration t = ros::Time::now() - t1;
    double t_s = t.toSec();
    // std::cout<<t_s<<"\t";

    Trajectory_pos = getPosition(t_s);
    Trajectory_vel = getVelocity(t_s);
    Trajectory_acc = getAcceleration(t_s);

    Eigen::Vector3d pos_err = pos_current - Trajectory_pos;
    Eigen::Vector3d vel_err = ver_current - Trajectory_vel;


    Eigen::Vector3d k_pos,k_vel;
    k_pos << -8 ,-8 , -10;
    k_vel << -1.5, -3.3,  -3.3;

    Eigen::Vector4f quat_current;
    quat_current << pos.pose.orientation.w, pos.pose.orientation.x, 
                    pos.pose.orientation.y, pos.pose.orientation.z;
    Eigen::Matrix3d rotation_current;
    rotation_current = quat2RotMatrix(quat_current);
    double thrust;

    double psi = 0;
    Eigen::Vector3d a_fb = k_pos.asDiagonal()*pos_err + k_vel.asDiagonal()*vel_err;
    
    if(a_fb.norm() > 8) 
        a_fb = (8 / a_fb.norm()) * a_fb;
    
    const Eigen::Vector3d a_des = a_fb + g*Eigen::Vector3d(0,0,1) + Trajectory_acc;
    thrust = 0.05*a_des.dot(rotation_current*Eigen::Vector3d(0,0,1));

    Eigen::Vector3d proj_xb_des(cos(psi),sin(psi),0);
    Eigen::Vector3d zb_des = a_des/a_des.norm(); 
    Eigen::Vector3d yb_des = zb_des.cross(proj_xb_des)/(zb_des.cross(proj_xb_des).norm());
    Eigen::Vector3d xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();
    Eigen::Matrix3d R_des;
    R_des << xb_des(0), yb_des(0), zb_des(0),
            xb_des(1), yb_des(1), zb_des(1),
            xb_des(2), yb_des(2), zb_des(2);
    // std::cout<<thrust<<"\t"<<pos.pose.position.z<<std::endl;

    Eigen::Vector4f quat_des = rot2Quaternion(R_des);
    Eigen::Vector4f quat_curr(quat_current);

    Eigen::Vector3d ratecmd = attcontroller(quat_des,quat_curr);
    local_attitude_target.body_rate.x = ratecmd(0);
    local_attitude_target.body_rate.y = ratecmd(1);
    local_attitude_target.body_rate.z = ratecmd(2);
    local_attitude_target.thrust = std::max(0.0,std::min(1.0, thrust+0.1));
    local_attitude_target.type_mask = 0b10111000;

    //**********
    // std::cout<<ratecmd.transpose()<<"\t"<<std::max(0.0,std::min(1.0, thrust+0.1))<<std::endl;
    //**********
}

int init_finished(geometry_msgs::PoseStamped pos)
{  
    static uint16_t count = 0;
    float error_z = local_pos_position.pose.position.z - pos.pose.position.z;
    if(fabs(error_z)<0.2)
    {
        count ++;
    }
    else{
        count = 0;
    }
    if(count>50)
    {
        return 1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    uint8_t code_step = 0;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Publisher local_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude",10);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Publisher local_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpointraw_attitude",10);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,local_pos_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local",10,local_vel_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(125.0);  //100hz

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    Eigen::Vector3d EularAngle_current;
    ros::Time Circle_begin_t;
    uint8_t Circle_T_record = 1;
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
            else{
                if(init_finished(pose) || code_step){
                    if(Circle_T_record){
                        Circle_begin_t = ros::Time::now();
                        Circle_T_record = 0;
                    }
                    Circle_trajectory(local_pos_position,Circle_begin_t);
                    local_attitude_pub.publish(local_attitude_target);
                    code_step = 1;
                }
                else{
                    code_step = 0;
                }
            }
        }
        if(!code_step){
            local_pos_pub.publish(pose);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}