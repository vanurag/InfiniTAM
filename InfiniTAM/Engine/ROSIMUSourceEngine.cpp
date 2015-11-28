/*
 * ROSIMUSourceEngine.cpp
 *
 *  Created on: Nov 23, 2015
 *      Author: anurag
 *
 *  Using JPL's Quaternion format: https://claraty.jpl.nasa.gov/man/software/development/conventions/standards_docs/unadopted/JPL_Quaternions_Breckenridge.pdf
 */


#include "ROSIMUSourceEngine.h"

#include "../Utils/FileUtils.h"

#include <stdio.h>

using namespace InfiniTAM::Engine;

cv::viz::Viz3d ROSIMUSourceEngine::viz_window = cv::viz::Viz3d("Pose Viewer");
cv::Affine3f ROSIMUSourceEngine::viz_cam_pose = cv::Affine3f();

ROSIMUSourceEngine::ROSIMUSourceEngine(const char *imuMask) : IMUSourceEngine(imuMask),
    viz_key_event(cv::viz::KeyboardEvent::Action::KEY_DOWN, "A", cv::viz::KeyboardEvent::ALT, 1)
{
  strncpy(this->imuMask, imuMask, BUF_SIZE);
  ros::master::getTopics(master_topics);

  for (auto topic : master_topics) {
    if (topic.name == imuMask) {
      if (topic.datatype == std::string("nav_msgs/Odometry")) {
        sub_pose_ = node_.subscribe(node_.resolveName(imuMask), 1,
                                    &ROSIMUSourceEngine::ROSOdometryCallback, this);
        break;
      }
      if (topic.datatype == std::string("sensor_msgs/Imu")) {
        sub_pose_ = node_.subscribe(node_.resolveName(imuMask), 1,
                                    &ROSIMUSourceEngine::ROSIMUCallback, this);
        break;
      }
      if (topic.datatype == std::string("geometry_msgs/TransformStamped")) {
        sub_pose_ = node_.subscribe(node_.resolveName(imuMask), 1,
                                    &ROSIMUSourceEngine::ROSTFCallback, this);
        break;
      }
    }
  }
  cached_imu = NULL;

  // Add camera coordinate axes visualization widget
  viz_window.setWindowSize(cv::Size(600, 600));
  viz_window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem(100.0));
  viz_window.registerKeyboardCallback(VizKeyboardCallback);
}

void ROSIMUSourceEngine::ROSOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Odometry Orientation x: [%f], y: [%f], z: [%f], w: [%f]",
           msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
           msg->pose.pose.orientation.w);
  quat2ITMIMU(Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
}

void ROSIMUSourceEngine::ROSIMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("IMU Orientation x: [%f], y: [%f], z: [%f], w: [%f]",
           msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  quat2ITMIMU(Quaternion(msg->orientation.x, msg->orientation.y,
                         msg->orientation.z, msg->orientation.w));
}

void ROSIMUSourceEngine::ROSTFCallback(
    const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  ROS_INFO("TF Orientation x: [%f], y: [%f], z: [%f], w: [%f]",
           msg->transform.rotation.x, msg->transform.rotation.y,
           msg->transform.rotation.z, msg->transform.rotation.w);
  quat2ITMIMU(Quaternion(msg->transform.rotation.x, msg->transform.rotation.y,
                         msg->transform.rotation.z, msg->transform.rotation.w));
}

// conversion from quaternion to rotation matrix
void ROSIMUSourceEngine::quat2ITMIMU(const Quaternion imu_pose) {

  // TODO(vanurag): make this user definable
  Quaternion q_cam_to_imu(-0.0439623792008964, -0.07685149594247381,
                          -0.9957522559683859, 0.025274394156932774);
  Quaternion cam_pose = imu_pose * q_cam_to_imu;
  cached_imu = new ITMIMUMeasurement();

  // JPL -> ORUtils Matrix
  cached_imu->R.m00 = pow(cam_pose.w, 2) + pow(cam_pose.x, 2) - 0.5;
  cached_imu->R.m10 = cam_pose.x*cam_pose.y + cam_pose.z*cam_pose.w;
  cached_imu->R.m20 = cam_pose.x*cam_pose.z - cam_pose.y*cam_pose.w;
  cached_imu->R.m01 = cam_pose.x*cam_pose.y - cam_pose.z*cam_pose.w;
  cached_imu->R.m11 = pow(cam_pose.y, 2) + pow(cam_pose.w, 2) - 0.5;
  cached_imu->R.m21 = cam_pose.y*cam_pose.z + cam_pose.x*cam_pose.w;
  cached_imu->R.m02 = cam_pose.x*cam_pose.z + cam_pose.y*cam_pose.w;
  cached_imu->R.m12 = cam_pose.y*cam_pose.z - cam_pose.x*cam_pose.w;
  cached_imu->R.m22 = pow(cam_pose.w, 2) + pow(cam_pose.z, 2) - 0.5;

  // Non-JPL --> ORUtils Matrix
//  cached_imu->R.m00 = 1.0 - 2*pow(cam_pose.y, 2) - 2*pow(cam_pose.z, 2);
//  cached_imu->R.m10 = 2.0*cam_pose.x*cam_pose.y - 2.0*cam_pose.z*cam_pose.w;
//  cached_imu->R.m20 = 2.0*cam_pose.x*cam_pose.z + 2.0*cam_pose.y*cam_pose.w;
//  cached_imu->R.m01 = 2.0*cam_pose.x*cam_pose.y + 2.0*cam_pose.z*cam_pose.w;
//  cached_imu->R.m11 = 1.0 - 2*pow(cam_pose.x, 2) - 2*pow(cam_pose.z, 2);
//  cached_imu->R.m21 = 2.0*cam_pose.y*cam_pose.z - 2.0*cam_pose.x*cam_pose.w;
//  cached_imu->R.m02 = 2.0*cam_pose.x*cam_pose.z - 2.0*cam_pose.y*cam_pose.w;
//  cached_imu->R.m12 = 2.0*cam_pose.y*cam_pose.z + 2.0*cam_pose.x*cam_pose.w;
//  cached_imu->R.m22 = 1.0 - 2*pow(cam_pose.x, 2) - 2*pow(cam_pose.y, 2);

  VisualizePose();

}

void ROSIMUSourceEngine::VisualizePose() {

  // Construct pose
  cv::Mat pose_mat(3, 3, CV_32F);
  float* mat_pointer = (float*)pose_mat.data;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      mat_pointer[3*row + col] = cached_imu->R(col, row);
    }
  }
  viz_cam_pose.rotation(pose_mat);
  viz_cam_pose.translate(cv::Vec3f(0.0, 0.0, 0.0));
  viz_window.setWidgetPose("Coordinate Widget", viz_cam_pose);
  viz_window.spinOnce(1, true);
}

bool ROSIMUSourceEngine::hasMoreMeasurements(void)
{
  return (cached_imu != NULL);
}

void ROSIMUSourceEngine::getMeasurement(ITMIMUMeasurement *imu)
{
  if (cached_imu != NULL)
  {
    ROS_INFO("Using IMU data...");
    imu->R = cached_imu->R;
    delete cached_imu;
    cached_imu = NULL;
  }
}


