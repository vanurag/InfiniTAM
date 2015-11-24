/*
 * VISensorIMUSourceEngine.cpp
 *
 *  Created on: Nov 23, 2015
 *      Author: anurag
 */


#include "ROSIMUSourceEngine.h"

#include "../Utils/FileUtils.h"

#include <stdio.h>

using namespace InfiniTAM::Engine;

VISensorIMUSourceEngine::VISensorIMUSourceEngine(const char *imuMask) : IMUSourceEngine(imuMask)
{
  strncpy(this->imuMask, imuMask, BUF_SIZE);
  ros::master::getTopics(master_topics);

  for (auto topic : master_topics) {
    if (topic.name == imuMask) {
      if (topic.datatype == std::string("nav_msgs/Odometry")) {
        sub_pose_ = node_.subscribe(node_.resolveName(imuMask), 1,
                                    &VISensorIMUSourceEngine::VISensorOdometryCallback, this);
        break;
      }
      if (topic.datatype == std::string("sensor_msgs/Imu")) {
        sub_pose_ = node_.subscribe(node_.resolveName(imuMask), 1,
                                    &VISensorIMUSourceEngine::VISensorIMUCallback, this);
        break;
      }
      if (topic.datatype == std::string("geometry_msgs/TransformStamped")) {
        sub_pose_ = node_.subscribe(node_.resolveName(imuMask), 1,
                                    &VISensorIMUSourceEngine::VISensorTFCallback, this);
        break;
      }
    }
  }
  cached_imu = NULL;
}

void VISensorIMUSourceEngine::VISensorOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Odometry Orientation x: [%f], y: [%f], z: [%f], w: [%f]",
           msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
           msg->pose.pose.orientation.w);
  quat2ITMIMU(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
              msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

void VISensorIMUSourceEngine::VISensorIMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("IMU Orientation x: [%f], y: [%f], z: [%f], w: [%f]",
           msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  quat2ITMIMU(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

void VISensorIMUSourceEngine::VISensorTFCallback(
    const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  ROS_INFO("TF Orientation x: [%f], y: [%f], z: [%f], w: [%f]",
           msg->transform.rotation.x, msg->transform.rotation.y,
           msg->transform.rotation.z, msg->transform.rotation.w);
  quat2ITMIMU(msg->transform.rotation.x, msg->transform.rotation.y,
              msg->transform.rotation.z, msg->transform.rotation.w);
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


