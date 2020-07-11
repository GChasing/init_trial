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

#define minimum_snap_Row 9
#define minimum_snap_Col 24
#define R_circle 2
#define mass 1.5
#define g 9.8
#define attctrl_tau_ 0.1
double traj_omega_ = 3;

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

Eigen::Vector3d traj_radial_(1,0,0);
Eigen::Vector3d traj_axis_(0,0,1);
Eigen::Vector3d traj_origin_(0,0,2);

mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_pos_position;
Eigen::Vector3d pos_current;
geometry_msgs::TwistStamped local_vel_current;
Eigen::Vector3d ver_current;
mavros_msgs::AttitudeTarget local_attitude_target;

#endif