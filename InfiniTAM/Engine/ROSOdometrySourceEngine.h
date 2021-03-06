/*
 * ROSOdometrySourceEngine.h
 *
 *  Created on: Feb 19, 2016
 *      Author: anurag
 */

#ifndef INFINITAM_INFINITAM_ENGINE_ROSODOMETRYSOURCEENGINE_H_
#define INFINITAM_INFINITAM_ENGINE_ROSODOMETRYSOURCEENGINE_H_

#pragma once

#include "OdometrySourceEngine.h"
#include "../ITMLib/ITMLib.h"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/viz/vizcore.hpp>
#include <opencv2/viz/types.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

namespace InfiniTAM
{
  namespace Engine
  {
    class ROSOdometrySourceEngine : public OdometrySourceEngine
    {
    private:
      static const int BUF_SIZE = 2048;
      char odomMask[BUF_SIZE];

      ros::NodeHandle node_;
      ros::Subscriber sub_pose_;
      ros::master::V_TopicInfo master_topics;

      // Visualization
      cv::viz::KeyboardEvent viz_key_event;
      static cv::viz::Viz3d viz_window;
      static cv::Affine3f viz_cam_pose;
      static void VizKeyboardCallback(const cv::viz::KeyboardEvent&, void*) {
        std::cout << "Setting VIZ viewing angle to camera's viewing direction" << std::endl;
        cv::Affine3f viz_viewer_pose = viz_cam_pose;
        viz_viewer_pose = viz_viewer_pose.translate(cv::Vec3f(0.0, 0.0, 10.0));
        viz_window.setViewerPose(viz_viewer_pose);
      }
      void VisualizePose();

    protected:
      ITMOdometryMeasurement *cached_odom;
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
      // conversion from quaternion to ITM Odometry Measurement
      void quat2ITMOdom(const Quaternion q);
      void ROSOdometryCallback_Odom(const nav_msgs::Odometry::ConstPtr& msg);
      void ROSTFCallback_Odom(const geometry_msgs::TransformStamped::ConstPtr& msg);
      void ROSPoseCallback_Odom(const geometry_msgs::PoseStamped::ConstPtr& msg);

    public:
      ROSOdometrySourceEngine(const char *odomMask);
      ROSOdometrySourceEngine();
      virtual ~ROSOdometrySourceEngine() { }

      virtual bool hasMoreMeasurements(void);
      virtual void getMeasurement(ITMOdometryMeasurement *imu);
    };
  }
}

#endif /* INFINITAM_INFINITAM_ENGINE_ROSODOMETRYSOURCEENGINE_H_ */
