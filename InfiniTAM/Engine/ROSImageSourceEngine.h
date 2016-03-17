/*
 * ROSImageSourceEngine.h
 *
 *  Created on: Mar 17, 2016
 *      Author: anurag
 */

#ifndef INFINITAM_INFINITAM_ENGINE_ROSIMAGESOURCEENGINE_H_
#define INFINITAM_INFINITAM_ENGINE_ROSIMAGESOURCEENGINE_H_

#pragma once

#include "ImageSourceEngine.h"

#include <stdio.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <time.h>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
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

namespace InfiniTAM
{
  namespace Engine
  {
    /** This class provides an interface for reading from the
        ROS Image topics
    */
    class ROSImageSourceEngine : public ImageSourceEngine
    {
    private:
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

      void CallBackFunction(const sensor_msgs::ImageConstPtr rgb_msg,
                            const sensor_msgs::ImageConstPtr depth_msg);

      char timestamp_[16];
      cv::Mat rgb_, depth_;
      ros::Subscriber sub_rgb_;
      ros::NodeHandle node_;
      message_filters::Subscriber<sensor_msgs::Image> mf_sub_rgb_, mf_sub_depth_;
      message_filters::Synchronizer<MySyncPolicy> sync_;
      Vector2i imageSize_d_, imageSize_rgb_;
      bool colorAvailable_, depthAvailable_;
    public:
      ROSImageSourceEngine(const char *calibFilename, const char *rgbTopic, const char *depthTopic,
                           const Vector2i rgbSize, const Vector2i depthSize);
      ~ROSImageSourceEngine() {};

      bool hasMoreImages(void);
      void getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage);
      Vector2i getDepthImageSize(void);
      Vector2i getRGBImageSize(void);
    };
  }
}


#endif /* INFINITAM_INFINITAM_ENGINE_ROSIMAGESOURCEENGINE_H_ */
