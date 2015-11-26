/*
 * ROSIMUSourceEngine.h
 *
 *  Created on: Nov 23, 2015
 *      Author: anurag
 */

#ifndef INFINITAM_INFINITAM_ENGINE_ROSIMUSOURCEENGINE_H_
#define INFINITAM_INFINITAM_ENGINE_ROSIMUSOURCEENGINE_H_

#pragma once

#include "IMUSourceEngine.h"
#include "../ITMLib/ITMLib.h"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

namespace InfiniTAM
{
  namespace Engine
  {
    class ROSIMUSourceEngine : public IMUSourceEngine
    {
    private:
      static const int BUF_SIZE = 2048;
      char imuMask[BUF_SIZE];

      ITMIMUMeasurement *cached_imu;

      ros::NodeHandle node_;
      ros::Subscriber sub_pose_;
      ros::master::V_TopicInfo master_topics;

      // conversion from quaternion to ITM IMU Measurement
      void quat2ITMIMU(const double qx, const double qy, const double qz, const double qw);
      void ROSOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
      void ROSIMUCallback(const sensor_msgs::Imu::ConstPtr& msg);
      void ROSTFCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);

    public:
      ROSIMUSourceEngine(const char *imuMask);
      virtual ~ROSIMUSourceEngine() { }

      virtual bool hasMoreMeasurements(void);
      virtual void getMeasurement(ITMIMUMeasurement *imu);
    };
  }
}



#endif /* INFINITAM_INFINITAM_ENGINE_ROSIMUSOURCEENGINE_H_ */
