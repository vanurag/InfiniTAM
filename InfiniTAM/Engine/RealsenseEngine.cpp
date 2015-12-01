/*
 * RealsenseEngine.cpp
 *
 *  Created on: Nov 18, 2015
 *      Author: anurag
 */


#include "RealsenseEngine.h"

#include "../Utils/FileUtils.h"

using namespace InfiniTAM::Engine;

RealsenseEngine::RealsenseEngine(const char *calibFilename) : ImageSourceEngine(calibFilename),
    mf_sub_rgb_(node_, "/camera/color/image_raw", 1),
    mf_sub_depth_(node_, "/camera/depth/image_raw", 1),
    sync_(MySyncPolicy(10), mf_sub_rgb_, mf_sub_depth_)
{
  imageSize_d_ = Vector2i(480, 360);
  imageSize_rgb_ = Vector2i(640, 480);
  colorAvailable_ = true;
  depthAvailable_ = true;

  sync_.registerCallback(boost::bind(&RealsenseEngine::RealsenseCallBackFunction, this, _1, _2));
}

void RealsenseEngine::RealsenseCallBackFunction(const sensor_msgs::ImageConstPtr rgb_msg,
                                                const sensor_msgs::ImageConstPtr depth_msg)
{
  if (rgb_.empty()) {
    rgb_.create(cv::Size(rgb_msg->width, rgb_msg->height), CV_8UC3);
  }
  if (depth_.empty()) {
    depth_.create(cv::Size(depth_msg->width, depth_msg->height), CV_16UC1);
  }
  rgb_ = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8)->image;
  depth_ = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
  imageSize_rgb_ = Vector2i(rgb_.cols, rgb_.rows);
  imageSize_d_ = Vector2i(depth_.cols, depth_.rows);
  colorAvailable_ = true;
  depthAvailable_ = true;
}

void RealsenseEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
  ros::spinOnce();
  Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
  short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
  if (colorAvailable_&&depthAvailable_)
  {

    // RGB data
    uchar* rgb_pointer = (uchar*)rgb_.data;
    for (int j = 0; j < imageSize_rgb_.y; ++j)
    {
      for (int i = 0; i < imageSize_rgb_.x; ++i)
      {
        rgb[j*imageSize_rgb_.x + i].b   = rgb_pointer[j*3*imageSize_rgb_.x + 3*i];
        rgb[j*imageSize_rgb_.x + i].g = rgb_pointer[j*3*imageSize_rgb_.x + 3*i +1];
        rgb[j*imageSize_rgb_.x + i].r = rgb_pointer[j*3*imageSize_rgb_.x + 3*i + 2];
        rgb[j*imageSize_rgb_.x + i].a = 255.0;
      }
    }

    // Depth data
    unsigned short* depth_pointer = (unsigned short*)depth_.data;
    for (int i = 0; i < imageSize_d_.x * imageSize_d_.y; ++i) {
      depth[i] = (short)depth_pointer[i];
    }
  }else{
    memset(depth, 0, rawDepthImage->dataSize * sizeof(short));
    memset(rgb, 0, rgbImage->dataSize * sizeof(Vector4u));
  }
  return /*true*/;
}

bool RealsenseEngine::hasMoreImages(void) { return true; }
Vector2i RealsenseEngine::getDepthImageSize(void) { return imageSize_d_; }
Vector2i RealsenseEngine::getRGBImageSize(void) { return imageSize_rgb_; }


