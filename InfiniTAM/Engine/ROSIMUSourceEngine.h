/*
 * ROSIMUSourceEngine.h
 *
 *  Created on: Nov 23, 2015
 *      Author: anurag
 *
 *  Using JPL's Quaternion format: https://claraty.jpl.nasa.gov/man/software/development/conventions/standards_docs/unadopted/JPL_Quaternions_Breckenridge.pdf
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
#include <opencv2/viz/vizcore.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

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

      /// Create visualization window
      cv::viz::Viz3d myWindow;

      struct Quaternion {
        double x;
        double y;
        double z;
        double w;

        Quaternion() : x(0.0), y(0.0), z(0.0), w(1.0) {}
        Quaternion(const double a, const double b, const double c, const double d) {
          x = a;
          y = b;
          z = c;
          w = d;
        }

        void operator=(const Quaternion& assign) {
          x = assign.x;
          y = assign.y;
          z = assign.z;
          w = assign.w;
        }

        Quaternion operator*(const Quaternion& right) const {
          Quaternion result;
          result.x = w*right.x + z*right.y - y*right.z + x*right.w;
          result.y = -z*right.x + w*right.y + x*right.z + y*right.w;
          result.z = y*right.x - x*right.y + w*right.z + z*right.w;
          result.w = -x*right.x - y*right.y - z*right.z + w*right.w;

          return result;
        }
      };

      // conversion from quaternion to ITM IMU Measurement
      void quat2ITMIMU(const Quaternion q);
      void ROSOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
      void ROSIMUCallback(const sensor_msgs::Imu::ConstPtr& msg);
      void ROSTFCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
      void VisualizePose();

    public:
      ROSIMUSourceEngine(const char *imuMask);
      virtual ~ROSIMUSourceEngine() { }

      virtual bool hasMoreMeasurements(void);
      virtual void getMeasurement(ITMIMUMeasurement *imu);
    };
  }
}



#endif /* INFINITAM_INFINITAM_ENGINE_ROSIMUSOURCEENGINE_H_ */
