/*
 * ROSBagSourceEngine.h
 *
 *  Created on: Mar 18, 2016
 *      Author: anurag
 */

#ifndef INFINITAM_INFINITAM_ENGINE_ROSBAGSOURCEENGINE_H_
#define INFINITAM_INFINITAM_ENGINE_ROSBAGSOURCEENGINE_H_

#pragma once

#include "ROSImageSourceEngine.h"
#include "ROSIMUSourceEngine.h"
#include "ROSOdometrySourceEngine.h"
#include "../ITMLib/ITMLib.h"

#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <iostream>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <opencv2/viz/types.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace InfiniTAM
{
  namespace Engine
  {
    /** This class provides an interface for reading Images and Pose from
        ROS Bag topics
    */
    class ROSBagSourceEngine;
    class ROSBagImageSourceEngine : public ROSImageSourceEngine
    {
    private:
      ROSBagSourceEngine* source_engine_;   // pointer to the owner source engine
      bool got_new_image_pair_ = false; // indicates if a new <rgb, depth> measurement is available
      rosbag::Bag bag_;
      rosbag::View bag_view_;
      rosbag::View::iterator current_bag_pos_;
      std::string rgb_topic_, depth_topic_, pose_topic_;
      sensor_msgs::ImageConstPtr rgb_msg_, depth_msg_;
      nav_msgs::Odometry::ConstPtr odom_msg_;
      sensor_msgs::Imu::ConstPtr imu_msg_;
      geometry_msgs::TransformStamped::ConstPtr tf_msg_;
      geometry_msgs::PoseStamped::ConstPtr pose_msg_;
    public:
      ROSBagImageSourceEngine(
          ROSBagSourceEngine& source_engine, const char *calibFilename, const char *bagFileName,
          const char *rgbTopic, const char *depthTopic,
          const Vector2i rgbSize, const Vector2i depthSize);
      ROSBagImageSourceEngine(
          ROSBagSourceEngine& source_engine, const char *calibFilename, const char *bagFileName,
          const char *rgbTopic, const char *depthTopic, const char *poseTopic,
          const Vector2i rgbSize, const Vector2i depthSize);
      ~ROSBagImageSourceEngine() {};

      bool hasMoreImages(void);
      void getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage);
      Vector2i getDepthImageSize(void);
      Vector2i getRGBImageSize(void);
    };

    class ROSBagIMUSourceEngine : public ROSIMUSourceEngine
    {
    protected:
      void OdometryCallback_IMU(const nav_msgs::Odometry::ConstPtr& msg) {this->ROSOdometryCallback_IMU(msg);}
      void IMUCallback_IMU(const sensor_msgs::Imu::ConstPtr& msg) {this->ROSIMUCallback_IMU(msg);}
      void TFCallback_IMU(const geometry_msgs::TransformStamped::ConstPtr& msg) {this->ROSTFCallback_IMU(msg);}
      void PoseCallback_IMU(const geometry_msgs::PoseStamped::ConstPtr& msg) {this->ROSPoseCallback_IMU(msg);}
    public:
      friend class ROSBagImageSourceEngine;
      ROSBagIMUSourceEngine();
      virtual ~ROSBagIMUSourceEngine() { }

      virtual bool hasMoreMeasurements(void);
      virtual void getMeasurement(ITMIMUMeasurement *imu);
    };

    class ROSBagOdometrySourceEngine : public ROSOdometrySourceEngine
    {
    protected:
      void OdometryCallback_Odom(const nav_msgs::Odometry::ConstPtr& msg) {this->ROSOdometryCallback_Odom(msg);}
      void TFCallback_Odom(const geometry_msgs::TransformStamped::ConstPtr& msg) {this->ROSTFCallback_Odom(msg);}
      void PoseCallback_Odom(const geometry_msgs::PoseStamped::ConstPtr& msg) {this->ROSPoseCallback_Odom(msg);}
    public:
      friend class ROSBagImageSourceEngine;
      ROSBagOdometrySourceEngine();
      virtual ~ROSBagOdometrySourceEngine() { }

      virtual bool hasMoreMeasurements(void);
      virtual void getMeasurement(ITMOdometryMeasurement *imu);
    };

    class ROSBagSourceEngine
    {
    private:
      // Visualization
      Matrix3f viz_cached_pose_;
//      cv::viz::KeyboardEvent viz_key_event;
//      static cv::viz::Viz3d viz_window;
//      static cv::Affine3f viz_pose;
//      static void VizKeyboardCallback(const cv::viz::KeyboardEvent&, void*) {
//        std::cout << "Setting VIZ viewing angle to camera's viewing direction" << std::endl;
//        cv::Affine3f viz_viewer_pose = viz_pose;
//        viz_viewer_pose = viz_viewer_pose.translate(cv::Vec3f(0.0, 0.0, 10.0));
//        viz_window.setViewerPose(viz_viewer_pose);
//      }
      void VisualizePose();

    public:
      friend class ROSBagImageSourceEngine;
      ROSBagImageSourceEngine* rosbag_image_source_engine;
      ROSBagIMUSourceEngine* rosbag_imu_source_engine;
      ROSBagOdometrySourceEngine* rosbag_odometry_source_engine;
      ROSBagSourceEngine(
          const char *calibFilename, const char *bagFileName,
          const char *rgbTopic, const char *depthTopic,
          const Vector2i rgbSize, const Vector2i depthSize);
      ROSBagSourceEngine(
          const char *calibFilename, const char *bagFileName,
          const char *rgbTopic, const char *depthTopic, const char *poseTopic,
          const Vector2i rgbSize, const Vector2i depthSize, const char *pose_type);
      ~ROSBagSourceEngine() {};
    };
  }
}



#endif /* INFINITAM_INFINITAM_ENGINE_ROSBAGSOURCEENGINE_H_ */
