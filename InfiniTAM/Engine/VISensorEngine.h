/*
 * VISensorEngine.h
 *
 *  Created on: Nov 17, 2015
 *      Author: anurag
 */

#ifndef INFINITAM_INFINITAM_ENGINE_VISENSORENGINE_H_
#define INFINITAM_INFINITAM_ENGINE_VISENSORENGINE_H_

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
        Skybotix VI-Sensor.
    */
    class VISensorEngine : public ImageSourceEngine
    {
    private:
      void VISensorCallBackFunction(const sensor_msgs::ImageConstPtr& msg_image_fused,
                                          const sensor_msgs::ImageConstPtr& msg_depthmap_fused);

      char timestamp_[16];
      cv::Mat rgb_, depth_;
      ros::NodeHandle node_, node_local_;
      message_filters::Subscriber<sensor_msgs::Image> mf_sub_image_fused_;
      message_filters::Subscriber<sensor_msgs::Image> mf_sub_depthmap_fused_;
      Vector2i imageSize_d_, imageSize_rgb_;
      bool colorAvailable_, depthAvailable_;
    public:
      VISensorEngine(const char *calibFilename);
      ~VISensorEngine() {};

      bool hasMoreImages(void);
      void getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage);
      Vector2i getDepthImageSize(void);
      Vector2i getRGBImageSize(void);
    };
  }
}

#endif /* INFINITAM_INFINITAM_ENGINE_VISENSORENGINE_H_ */
