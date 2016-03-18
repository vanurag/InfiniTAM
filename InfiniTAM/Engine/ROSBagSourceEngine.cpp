/*
 * ROSBagSourceEngine.cpp
 *
 *  Created on: Mar 18, 2016
 *      Author: anurag
 */


#include "ROSBagSourceEngine.h"

#include "../Utils/FileUtils.h"

using namespace InfiniTAM::Engine;

//cv::viz::Viz3d ROSBagSourceEngine::viz_window = cv::viz::Viz3d("IMU pose");
//cv::Affine3f ROSBagSourceEngine::viz_pose = cv::Affine3f();

ROSBagSourceEngine::ROSBagImageSourceEngine::ROSBagImageSourceEngine(
    const char *calibFilename, const Vector2i rgbSize, const Vector2i depthSize) :
        ROSImageSourceEngine(calibFilename, rgbSize, depthSize) { }
void ROSBagSourceEngine::ROSBagImageSourceEngine::getImages(
    ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage){

  for (; current_bag_pos_ != bag_view_.end() && !got_new_image_pair_; ++current_bag_pos_) {
    rosbag::MessageInstance& message_view = *current_bag_pos_;

    // Read ROS RGB Image message
    if (message_view.getTopic() == rgb_topic_) {
      rgb_msg_ = message_view.instantiate<sensor_msgs::Image>();
      CHECK(rgb_msg_ != nullptr)
          << "Wrong type on topic " << rgb_topic_
          << " expected sensor_msgs::Image";
    }

    // Read ROS Depth Image message
    if (message_view.getTopic() == depth_topic_) {
      depth_msg_ = message_view.instantiate<sensor_msgs::Image>();
      CHECK(depth_msg_ != nullptr)
          << "Wrong type on topic " << depth_topic_
          << " expected sensor_msgs::Image";
    }

    // Read ROS Pose message
    if (message_view.getTopic() == pose_topic_) {
      if (message_view.getDataType() == std::string("nav_msgs/Odometry")) {
        odom_msg_ = message_view.instantiate<nav_msgs::Odometry>();
        CHECK(odom_msg_ != nullptr)
            << "Wrong type on topic " << pose_topic_
            << " expected nav_msgs::Odometry";
        // callback
        if (rosbag_imu_source_engine != NULL) {
          rosbag_imu_source_engine.ROSOdometryCallback_IMU(odom_msg_);
        } else if (rosbag_odometry_source_engine != NULL) {
          rosbag_odometry_source_engine.ROSOdometryCallback_Odom(odom_msg_);
        }
      }
      if (message_view.getDataType() == std::string("sensor_msgs/Imu")) {
        imu_msg_ = message_view.instantiate<sensor_msgs::Imu>();
        CHECK(imu_msg_ != nullptr)
            << "Wrong type on topic " << pose_topic_
            << " expected sensor_msgs::Imu";
        // callback
        if (rosbag_imu_source_engine != NULL) {
          rosbag_imu_source_engine.ROSIMUCallback_IMU(imu_msg_);
        }
      }
      if (message_view.getDataType() == std::string("geometry_msgs/TransformStamped")) {
        tf_msg_ = message_view.instantiate<geometry_msgs::TransformStamped>();
        CHECK(tf_msg_ != nullptr)
            << "Wrong type on topic " << pose_topic_
            << " expected geometry_msgs::TransformStamped";
        // callback
        if (rosbag_imu_source_engine != NULL) {
          rosbag_imu_source_engine.ROSTFCallback_IMU(tf_msg_);
        } else if (rosbag_odometry_source_engine != NULL) {
          rosbag_odometry_source_engine.ROSTFCallback_Odom(tf_msg_);
        }
      }
      if (message_view.getDataType() == std::string("geometry_msgs/PoseStamped")) {
        pose_msg_ = message_view.instantiate<geometry_msgs::PoseStamped>();
        CHECK(pose_msg_ != nullptr)
            << "Wrong type on topic " << pose_topic_
            << " expected geometry_msgs::PoseStamped";
        // callback
        if (rosbag_imu_source_engine != NULL) {
          rosbag_imu_source_engine.ROSPoseCallback_IMU(pose_msg_);
        } else if (rosbag_odometry_source_engine != NULL) {
          rosbag_odometry_source_engine.ROSPoseCallback_Odom(pose_msg_);
        }
      }

      // viz
      if (rosbag_imu_source_engine != NULL) {
        viz_cached_pose_ = rosbag_imu_source_engine.cached_imu->R;
      } else if (rosbag_odometry_source_engine != NULL) {
        viz_cached_pose_ = rosbag_odometry_source_engine.cached_odom->R;
      } else {
        viz_cached_pose_ = Matrix3f(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
      }
    }

    // check if RGB and Depth image msgs have close enough timestamps to consider as pair
    if (rgb_msg_ != nullptr && depth_msg_ != nullptr) {
      if (rgb_msg_->header.stamp.sec == depth_msg_->header.stamp.sec &&
          abs(rgb_msg_->header.stamp.nsec - depth_msg_->header.stamp.nsec) < 200000000) {
        got_new_image_pair_ = true;
        rosbag_image_source_engine.ROSImageCallback(rgb_msg_, depth_msg_);

        // fill up ITM data
        Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
        short *depth = rawDepthImage->GetData(MEMORYDEVICE_CPU);
        if (colorAvailable_ && depthAvailable_)
        {
          // RGB data
          uchar* rgb_pointer = (uchar*)rgb_.data;
          for (int j = 0; j < imageSize_rgb_.y; ++j)
          {
            for (int i = 0; i < imageSize_rgb_.x; ++i)
            {
              rgb[j*imageSize_rgb_.x + i].b = rgb_pointer[j*3*imageSize_rgb_.x + 3*i];
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
      }
    }
  } // seeking through bag until I get an image pair

  got_new_image_pair_ = false;
  rgb_msg_ = nullptr;
  depth_msg_ = nullptr;
  return /*true*/;
}
bool ROSBagSourceEngine::ROSBagImageSourceEngine::hasMoreImages(void) {
  return (current_bag_pos_ == bag_view_.end());
}
Vector2i ROSBagSourceEngine::ROSBagImageSourceEngine::getDepthImageSize(void) { return imageSize_d_; }
Vector2i ROSBagSourceEngine::ROSBagImageSourceEngine::getRGBImageSize(void) { return imageSize_rgb_; }



ROSBagSourceEngine::ROSBagIMUSourceEngine::ROSBagIMUSourceEngine() : ROSIMUSourceEngine() { }
bool ROSBagSourceEngine::ROSBagIMUSourceEngine::hasMoreMeasurements(void)
{
  return (cached_imu != NULL);
}
void ROSBagSourceEngine::ROSBagIMUSourceEngine::getMeasurement(ITMIMUMeasurement *imu)
{
  if (cached_imu != NULL)
  {
    ROS_INFO("Using IMU data...");
    imu->R = cached_imu->R;
    delete cached_imu;
    cached_imu = NULL;
  }
}



ROSBagSourceEngine::ROSBagOdometrySourceEngine::ROSBagOdometrySourceEngine() :
    ROSOdometrySourceEngine() { }
bool ROSBagSourceEngine::ROSBagOdometrySourceEngine::hasMoreMeasurements(void)
{
  return (cached_odom != NULL);
}
void ROSBagSourceEngine::ROSBagOdometrySourceEngine::getMeasurement(ITMOdometryMeasurement *odom)
{
  if (cached_odom != NULL)
  {
    ROS_INFO("Using ODOM data...");
    odom->R = cached_odom->R;
    odom->t = cached_odom->t;
    delete cached_odom;
    cached_odom = NULL;
  }
}



ROSBagSourceEngine::ROSBagSourceEngine(
    const char *bagFileName, const char *rgbTopic, const char *depthTopic, const char *poseTopic) :
        rgb_topic_(rgbTopic), depth_topic_(depthTopic), pose_topic_(poseTopic)//,
//        viz_key_event(cv::viz::KeyboardEvent::Action::KEY_DOWN, "A", cv::viz::KeyboardEvent::ALT, 1)
{
  rosbag_image_source_engine = NULL;
  rosbag_imu_source_engine = NULL;
  rosbag_odometry_source_engine = NULL;
  rgb_msg_ = nullptr;
  depth_msg_ = nullptr;

  bag_.open(bagFileName, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(rgb_topic_);
  topics.push_back(depth_topic_);
  topics.push_back(pose_topic_);

  bag_view_.addQuery(bag_, rosbag::TopicQuery(topics));
  current_bag_pos_ = bag_view_.begin();
  viz_cached_pose_ == NULL;

//  // Add camera coordinate axes visualization widget
//  viz_window.setWindowSize(cv::Size(600, 600));
////  viz_window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem(200.0));
////  viz_window.showWidget("Test Sphere", cv::viz::WSphere(cv::Point3f(100.0, 0.0, 0.0), 5.0));
//  viz_window.showWidget("Camera Widget", cv::viz::WCoordinateSystem(100.0));
//  viz_window.registerKeyboardCallback(VizKeyboardCallback);
}
void ROSBagSourceEngine::VisualizePose() {

  if (viz_cached_pose_ != NULL) {
    // Construct pose
    cv::Mat pose_mat(3, 3, CV_32F);
    float* mat_pointer = (float*)pose_mat.data;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        mat_pointer[3*row + col] = viz_cached_pose_(col, row);
      }
    }
//    viz_pose.rotation(pose_mat);
//    viz_pose.translate(cv::Vec3f(0.0, 0.0, 0.0));
//    viz_window.setWidgetPose("Camera Widget", viz_pose);
//    viz_window.spinOnce(1, true);
  }
}
