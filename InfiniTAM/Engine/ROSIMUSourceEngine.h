/*
 * VISensorIMUSourceEngine.h
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

namespace InfiniTAM
{
  namespace Engine
  {
    class VISensorIMUSourceEngine : public IMUSourceEngine
    {
    private:
      static const int BUF_SIZE = 2048;
      char imuMask[BUF_SIZE];

      ITMIMUMeasurement *cached_imu;

      ros::NodeHandle node_;
      ros::Subscriber sub_imu_;

      // conversion from quaternion to ITM IMU Measurement
      void quat2ITMIMU(const double qx, const double qy, const double qz, const double qw);
      void VISensorIMUCallback(const sensor_msgs::Imu::ConstPtr& msg);

    public:
      VISensorIMUSourceEngine(const char *imuMask);
      virtual ~VISensorIMUSourceEngine() { }

      virtual bool hasMoreMeasurements(void);
      virtual void getMeasurement(ITMIMUMeasurement *imu);
    };
  }
}



#endif /* INFINITAM_INFINITAM_ENGINE_ROSIMUSOURCEENGINE_H_ */
