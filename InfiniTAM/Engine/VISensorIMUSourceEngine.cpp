/*
 * VISensorIMUSourceEngine.cpp
 *
 *  Created on: Nov 23, 2015
 *      Author: anurag
 */


#include "VISensorIMUSourceEngine.h"

#include "../Utils/FileUtils.h"

#include <stdio.h>

using namespace InfiniTAM::Engine;

VISensorIMUSourceEngine::VISensorIMUSourceEngine(const char *imuMask) : IMUSourceEngine(imuMask)
{
  strncpy(this->imuMask, imuMask, BUF_SIZE);
  sub_imu_ = node_.subscribe(node_.resolveName(imuMask), 1,
                             &VISensorIMUSourceEngine::VISensorIMUCallback, this);
  cached_imu = NULL;
}

void VISensorIMUSourceEngine::VISensorIMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("IMU Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  quat2ITMIMU(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

// conversion from quaternion to rotation matrix
void VISensorIMUSourceEngine::quat2ITMIMU(
    const double qx, const double qy, const double qz, const double qw) {

  cached_imu = new ITMIMUMeasurement();

  cached_imu->R.m00 = 1 - 2*pow(qy, 2) - 2*pow(qz, 2);
  cached_imu->R.m01 = 2*qx*qy - 2*qz*qw;
  cached_imu->R.m02 = 2*qx*qz + 2*qy*qw;
  cached_imu->R.m10 = 2*qx*qy + 2*qz*qw;
  cached_imu->R.m11 = 1 - 2*pow(qx, 2) - 2*pow(qz, 2);
  cached_imu->R.m12 = 2*qy*qz - 2*qx*qw;
  cached_imu->R.m20 = 2*qx*qz - 2*qy*qw;
  cached_imu->R.m21 = 2*qy*qz + 2*qx*qw;
  cached_imu->R.m22 = 1 - 2*pow(qx, 2) - 2*pow(qy, 2);
}

bool VISensorIMUSourceEngine::hasMoreMeasurements(void)
{
  return (cached_imu != NULL);
}

void VISensorIMUSourceEngine::getMeasurement(ITMIMUMeasurement *imu)
{
  if (cached_imu != NULL)
  {
    ROS_INFO("Using IMU data...");
    imu->R = cached_imu->R;
    delete cached_imu;
    cached_imu = NULL;
  }
}


