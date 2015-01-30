/*
 * c_slam_roamfree,
 *
 *
 * Copyright (C) 2014 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_slam_roamfree.
 *
 * c_slam_roamfree is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_slam_roamfree is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_slam_roamfree.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ImuHandler.h"

#include <iostream>
#include <Eigen/Dense>

#include <ros/ros.h>

using namespace ROAMestimation;

namespace roamfree_visualodometry {

ImuHandler::ImuHandler(FactorGraphFilter* filter) :
    filter(filter) {

  // -- get parameters

  double imu_N, imu_dt;

  ros::NodeHandle _node("~");

  if (!_node.getParam("IMU_nominal_period", imu_dt)) {
    ROS_FATAL("parameter IMU_nominal_period undefined");
  }

  if (!_node.getParam("IMU_N_integration_steps", imu_N)) {
    ROS_FATAL("parameter IMU_N_integration_steps undefined");
  }

  initialized = false;

  imu = new ROAMimu::IMUIntegralHandler(imu_N, imu_dt); // instantiate the new handler

  // init the sensor noises
  //imu->getSensorNoises() = Eigen::Matrix<double, 6, 6>::Identity();
  imu->getSensorNoises().setZero();
  imu->getSensorNoises().diagonal() << 0.0016, 0.0016, 0.0016, 1.666e-5, 1.666e-5, 1.666e-5;

  imu->setPredictorEnabled(true);
}

void ImuHandler::init(tf::Transform T_R_IMU_tf, tf::Transform T_W_R_tf,
    double t) {
  // Accelerometer and Gyroscope biases
  Eigen::VectorXd accBias(3);
  accBias << 0.0, 0.0, 0.0;
  Eigen::VectorXd gyroBias(3);
  gyroBias << 0.0, 0.0, 0.0;

  //get the imu wrt odom center (NB: the odometry must be camera centric)
  const tf::Quaternion& R_R_IMU = T_R_IMU_tf.getRotation();
  const tf::Vector3& t_R_IMU = T_R_IMU_tf.getOrigin();

  Eigen::VectorXd T_O_IMU(7);
  T_O_IMU << t_R_IMU[0], t_R_IMU[1], t_R_IMU[2], R_R_IMU.w(), R_R_IMU.x(), R_R_IMU.y(), R_R_IMU.z();

  //get the initial robot position
  const tf::Quaternion& R_W_O = T_W_R_tf.getRotation();
  const tf::Vector3& t_W_O = T_W_R_tf.getOrigin();

  Eigen::VectorXd x0(7);
  x0 << t_W_O[0], t_W_O[1], t_W_O[2], R_W_O.w(), R_W_O.x(), R_W_O.y(), R_W_O.z();

  imu->init(filter, "IMUintegral", T_O_IMU, accBias, false, gyroBias, true, x0,
      t);

}

void ImuHandler::addInertialMeasurement(double za[3], double zw[3]) {
  /*if (!initialized) {
   if (_magnetic_field_received) {
   initialize(t, za, _last_magnetic_field_z);
   }
   } else {
   imu->step(za, zw);
   }*/

  imu->step(za, zw);
}

void ImuHandler::addMagneticFieldMeasurement(double zh[3]) {
  /*memcpy(_last_magnetic_field_z, zh, 3 * sizeof(double));
   _magnetic_field_received = true;*/
}

/*void ImuHandler::initialize(double t, const double za[3], const double zh[3]) {
 // we have to initialize the IMUIntegralHandler

 Eigen::VectorXd accBias(3);  // Accelerometer and Gyroscope biases
 accBias << 0.0, 0.0, 0.0;
 Eigen::VectorXd gyroBias(3);
 gyroBias << 0.0, 0.0, 0.0;

 // Transformation from CAMERA to IMU
 tf::Quaternion R_C_IMU(0.5, -0.5, 0.5, 0.5);

 // (N.B. system is camera-centric so T_C_IMU = T_O_IMU;
 Eigen::VectorXd T_O_IMU(7);
 T_O_IMU << 0.0, 0.0, 0.0, R_C_IMU.w(), R_C_IMU.x(), R_C_IMU.y(), R_C_IMU.z();

 // determine the IMU orientation from gravity and magnetic field
 double imu_q[4];
 initializeOrientationFromImu(za, zh, imu_q);

 tf::Quaternion R_W_IMU(imu_q[1], imu_q[2], imu_q[3], imu_q[0]);

 tf::Quaternion R_W_C = R_W_IMU * R_C_IMU.inverse();

 Eigen::VectorXd x0(7); // initial robot position
 x0 << 0.0, 0.0, 0.0, R_W_C.w(), R_W_C.x(), R_W_C.y(), R_W_C.z();

 imu->init(filter, "IMUintegral", T_O_IMU, accBias, true, gyroBias, true, x0,
 t);

 initialized = true;
 }

 void ImuHandler::initializeOrientationFromImu(const double *a, const double *h,
 double *q) const {
 Eigen::Map<const Eigen::Vector3d> za(a);
 Eigen::Map<const Eigen::Vector3d> zh(h);

 Eigen::Vector3d v2 = za;
 Eigen::Vector3d v0 = -za.cross(zh);
 Eigen::Vector3d v1 = za.cross(v0);

 v0.normalize();
 v1.normalize();
 v2.normalize();

 q[0] = sqrt(1 + v0(0) + v1(1) + v2(2)) / 2.0;
 q[1] = (v2(1) - v1(2)) / (4.0 * q[0]);
 q[2] = (v0(2) - v2(0)) / (4.0 * q[0]);
 q[3] = (v1(0) - v0(1)) / (4.0 * q[0]);

 }*/

}
