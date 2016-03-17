/*
 * ROSImageSourceEngine.cpp
 *
 *  Created on: Mar 17, 2016
 *      Author: anurag
 */


#include "ROSImageSourceEngine.h"

#include "../Utils/FileUtils.h"

using namespace InfiniTAM::Engine;

ROSImageSourceEngine::ROSImageSourceEngine(
    const char *calibFilename, const char *rgbTopic, const char *depthTopic,
    const Vector2i rgbSize, const Vector2i depthSize) : ImageSourceEngine(calibFilename),
    mf_sub_rgb_(node_, rgbTopic, 10),
    mf_sub_depth_(node_, depthTopic, 10),
    sync_(MySyncPolicy(10), mf_sub_rgb_, mf_sub_depth_)
{
  imageSize_d_ = depthSize;
  imageSize_rgb_ = rgbSize;
  colorAvailable_ = false;
  depthAvailable_ = false;

  sync_.registerCallback(boost::bind(&ROSImageSourceEngine::CallBackFunction, this, _1, _2));
}

void ROSImageSourceEngine::CallBackFunction(const sensor_msgs::ImageConstPtr rgb_msg,
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

void ROSImageSourceEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
  while (!colorAvailable_ && !depthAvailable_) {
    std::cout << "waiting for ROS Image streams..." << std::endl;
    ros::spinOnce();
  }
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

bool ROSImageSourceEngine::hasMoreImages(void) { return true; }
Vector2i ROSImageSourceEngine::getDepthImageSize(void) { return imageSize_d_; }
Vector2i ROSImageSourceEngine::getRGBImageSize(void) { return imageSize_rgb_; }




