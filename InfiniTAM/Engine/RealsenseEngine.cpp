/*
 * RealsenseEngine.cpp
 *
 *  Created on: Nov 18, 2015
 *      Author: anurag
 */


#include "RealsenseEngine.h"

#include "../Utils/FileUtils.h"

using namespace InfiniTAM::Engine;

RealsenseEngine::RealsenseEngine(const char *calibFilename) : ImageSourceEngine(calibFilename)
{
  imageSize_d_ = Vector2i(0, 0);
  imageSize_rgb_ = Vector2i(0, 0);
  colorAvailable_ = true;
  depthAvailable_ = true;

  ros::NodeHandle node_;
  message_filters::Subscriber<sensor_msgs::Image> mf_sub_rgb_(
      node_, node_.resolveName("/camera/color/image_raw"), 100);
  message_filters::Subscriber<sensor_msgs::Image> mf_sub_depth_(
      node_, node_.resolveName("/camera/depth/image_raw"), 100);
  sub_rgb_ = node_.subscribe(node_.resolveName("/camera/depth/image_raw"), 1, &RealsenseEngine::BlaCallBackFunction, this);

//  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(
//      mf_sub_rgb_, mf_sub_depth_, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), mf_sub_rgb_, mf_sub_depth_);

  sync.registerCallback(boost::bind(&RealsenseEngine::RealsenseCallBackFunction, this, _1, _2));

}

void RealsenseEngine::BlaCallBackFunction(const sensor_msgs::ImageConstPtr rgb_msg)
{
  std::cout << "Im hereeeeeeeeeeeeeeeeee1111" << std::endl;
}

void RealsenseEngine::RealsenseCallBackFunction(const sensor_msgs::ImageConstPtr rgb_msg,
                                                const sensor_msgs::ImageConstPtr depth_msg)
{
  rgb_.create(cv::Size(rgb_msg->width, rgb_msg->height), CV_8UC3);
  depth_.create(cv::Size(depth_msg->width, depth_msg->height), CV_16UC1);
  rgb_ = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8)->image;
  depth_ = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO16)->image;
  imageSize_rgb_ = Vector2i(rgb_.cols, rgb_.rows);
  imageSize_d_ = Vector2i(depth_.cols, depth_.rows);
  colorAvailable_ = true;
  depthAvailable_ = true;
  std::cout << "real callback check: real callback check: real callback check: " << imageSize_rgb_.x << " " << imageSize_rgb_.y << std::endl;
}

void RealsenseEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{

  std::cout << std::endl <<  "before spin" << std::endl;
  ros::spinOnce();
  Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
  short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
  if (colorAvailable_&&depthAvailable_)
  {
    std::cout << "check: color and depth available..." << std::endl;
    std::cout << "real check: " << imageSize_rgb_.x << " " << imageSize_rgb_.y << std::endl;

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
    uchar* depth_pointer = (uchar*)depth_.data;
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


