/*
 * VISensorEngine.cpp
 *
 *  Created on: Nov 17, 2015
 *      Author: anurag
 */

#include "VISensorEngine.h"

#include "../Utils/FileUtils.h"

using namespace InfiniTAM::Engine;

VISensorEngine::VISensorEngine(const char *calibFilename) : ImageSourceEngine(calibFilename),
    mf_sub_rgb_(node_, "/cam0/image_raw", 100),
    mf_sub_depth_(node_, "/remode/depth", 100),
    sync_(MySyncPolicy(10), mf_sub_rgb_, mf_sub_depth_)
{
  imageSize_d_ = Vector2i(752, 480);
  imageSize_rgb_ = Vector2i(752, 480);

  colorAvailable_ = true;
  depthAvailable_ = true;

  sync_.registerCallback(boost::bind(&VISensorEngine::VISensorCallBackFunction, this, _1, _2));
}

void VISensorEngine::VISensorCallBackFunction(const sensor_msgs::ImageConstPtr rgb_msg,
                                              const sensor_msgs::ImageConstPtr depth_msg)
{
  if (rgb_.empty()) {
    rgb_.create(cv::Size(rgb_msg->width, rgb_msg->height), CV_8UC3);
  }
  if (depth_.empty()) {
    depth_.create(cv::Size(depth_msg->width, depth_msg->height), CV_32FC1);
  }
  rgb_ = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8)->image;
  depth_ = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
  imageSize_rgb_ = Vector2i(rgb_.cols, rgb_.rows);
  imageSize_d_ = Vector2i(depth_.cols, depth_.rows);
  colorAvailable_ = true;
  depthAvailable_ = true;
}

void VISensorEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{

  ros::spinOnce();
  Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
  short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
  if (colorAvailable_&&depthAvailable_) // in libFreenect2, both data are available or neither is available
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
    float* depth_pointer = (float*)depth_.data;
    for (int i = 0; i < imageSize_d_.x * imageSize_d_.y; ++i) {
      depth[i] = (short)depth_pointer[i];
    }
  }else{
    memset(depth, 0, rawDepthImage->dataSize * sizeof(short));
    memset(rgb, 0, rgbImage->dataSize * sizeof(Vector4u));
  }
  return /*true*/;
}

bool VISensorEngine::hasMoreImages(void) { return true; }
Vector2i VISensorEngine::getDepthImageSize(void) { return imageSize_d_; }
Vector2i VISensorEngine::getRGBImageSize(void) { return imageSize_rgb_; }


