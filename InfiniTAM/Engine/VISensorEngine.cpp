/*
 * VISensorEngine.cpp
 *
 *  Created on: Nov 17, 2015
 *      Author: anurag
 */

#include "VISensorEngine.h"

#include "../Utils/FileUtils.h"

using namespace InfiniTAM::Engine;

VISensorEngine::VISensorEngine(const char *calibFilename) : ImageSourceEngine(calibFilename), node_local_("~")
{
  imageSize_d_ = Vector2i(512, 424);
  imageSize_rgb_ = Vector2i(1920, 1080);

  colorAvailable_ = true;
  depthAvailable_ = true;

  // get timestamp
  time_t time_now;
  time_now = std::time(NULL);
  std::strftime(timestamp_,16,"%y%m%d_%H%M%S",std::localtime(&time_now));

  mf_sub_image_fused_.subscribe(node_, "/stereo_dense_reconstruction/image_fused", 1);
  mf_sub_depthmap_fused_.subscribe(node_, "/stereo_dense_reconstruction/depthmap_fused", 1);

  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(
      mf_sub_image_fused_, mf_sub_depthmap_fused_, 10);

  sync.registerCallback(boost::bind(&VISensorEngine::VISensorCallBackFunction, _1, _2));

}

void VISensorEngine::VISensorCallBackFunction(const sensor_msgs::ImageConstPtr& msg_image_fused,
                                              const sensor_msgs::ImageConstPtr& msg_depthmap_fused)
{
  rgb_.create(cv::Size(msg_image_fused->width, msg_image_fused->height), CV_8UC3);
  depth_.create(cv::Size(msg_depthmap_fused->width, msg_depthmap_fused->height), CV_16UC1);
  rgb_ = cv_bridge::toCvCopy(msg_image_fused, sensor_msgs::image_encodings::BGR8)->image;
  depth_ = cv_bridge::toCvCopy(msg_depthmap_fused, sensor_msgs::image_encodings::MONO16)->image;
  imageSize_rgb_ = Vector2i(rgb_.cols, rgb_.rows);
  imageSize_d_ = Vector2i(depth_.cols, depth_.rows);
}


void VISensorEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{

  ros::spinOnce();
  Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
  short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
  if (colorAvailable_&&depthAvailable_) // in libFreenect2, both data are available or neither is available
  {
    std::cout << "check: color and depth available..." << std::endl;

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
    uchar* depth_pointer = (uchar*)depth_;
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


