#ifndef _RL_LIB_H_
#define _RL_LIB_H_
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q);
void quat2eular_show(const Eigen::Quaterniond &quatf);
Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p);
Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R);
Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q);

#endif