/*
 * ROSOdometrySourceEngine.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: anurag
 */

#include "ROSOdometrySourceEngine.h"

#include "../Utils/FileUtils.h"

#include <stdio.h>

using namespace InfiniTAM::Engine;

cv::viz::Viz3d ROSOdometrySourceEngine::viz_window = cv::viz::Viz3d("ODOM pose");
cv::Affine3f ROSOdometrySourceEngine::viz_cam_pose = cv::Affine3f();

ROSOdometrySourceEngine::ROSOdometrySourceEngine(const char *odomMask) : OdometrySourceEngine(odomMask),
    viz_key_event(cv::viz::KeyboardEvent::Action::KEY_DOWN, "A", cv::viz::KeyboardEvent::ALT, 1)
{
  strncpy(this->odomMask, odomMask, BUF_SIZE);
  ros::master::getTopics(master_topics);

  for (auto topic : master_topics) {
    if (topic.name == odomMask) {
      if (topic.datatype == std::string("nav_msgs/Odometry")) {
        sub_pose_ = node_.subscribe(node_.resolveName(odomMask), 1,
                                    &ROSOdometrySourceEngine::ROSOdometryCallback, this);
        break;
      }
      if (topic.datatype == std::string("geometry_msgs/TransformStamped")) {
        sub_pose_ = node_.subscribe(node_.resolveName(odomMask), 1,
                                    &ROSOdometrySourceEngine::ROSTFCallback, this);
        break;
      }
    }
  }
  cached_odom = NULL;

  // Add camera coordinate axes visualization widget
  viz_window.setWindowSize(cv::Size(600, 600));
//  viz_window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem(200.0));
//  viz_window.showWidget("Test Sphere", cv::viz::WSphere(cv::Point3f(100.0, 0.0, 0.0), 5.0));
  viz_window.showWidget("Camera Widget", cv::viz::WCoordinateSystem(100.0));
  viz_window.registerKeyboardCallback(VizKeyboardCallback);
}

void ROSOdometrySourceEngine::ROSOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Odometry Orientation x: [%f], y: [%f], z: [%f], w: [%f]",
           msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
           msg->pose.pose.orientation.w);
  quat2ITMIMU(Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
  cached_odom->t.x = msg->pose.pose.position.x;
  cached_odom->t.y = msg->pose.pose.position.y;
  cached_odom->t.z = msg->pose.pose.position.z;
}

void ROSOdometrySourceEngine::ROSTFCallback(
    const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  ROS_INFO("TF Orientation x: [%f], y: [%f], z: [%f], w: [%f]",
           msg->transform.rotation.x, msg->transform.rotation.y,
           msg->transform.rotation.z, msg->transform.rotation.w);
  quat2ITMIMU(Quaternion(msg->transform.rotation.x, msg->transform.rotation.y,
                         msg->transform.rotation.z, msg->transform.rotation.w));
  cached_odom->t.x = msg->transform.translation.x;
  cached_odom->t.y = msg->transform.translation.y;
  cached_odom->t.z = msg->transform.translation.z;
}

// conversion from quaternion to rotation matrix
void ROSOdometrySourceEngine::quat2ITMIMU(const Quaternion odom_pose) {

  // TODO(vanurag): make this user definable
//  Quaternion q_cam_to_imu(-0.0439623792008964, -0.07685149594247381,
//                          -0.9957522559683859, 0.025274394156932774);
//  Quaternion cam_pose = imu_pose * q_cam_to_imu;
//  Quaternion q_imu_to_cam(0.0439623792008964, 0.07685149594247381,
//                          0.9957522559683859, 0.025274394156932774);
//  Quaternion cam_pose = q_imu_to_cam * imu_pose;
  cached_odom = new ITMOdometryMeasurement();

  // IMU Tracker only needs differential rotational changes.
  // So, no need to convert to camera reference frame.
  Quaternion cam_pose = odom_pose;
  // JPL Convention quaternion -> ORUtils Matrix
  cached_odom->R.m00 = pow(cam_pose.w, 2) + pow(cam_pose.x, 2) - 0.5;
  cached_odom->R.m01 = cam_pose.x*cam_pose.y + cam_pose.z*cam_pose.w;
  cached_odom->R.m02 = cam_pose.x*cam_pose.z - cam_pose.y*cam_pose.w;
  cached_odom->R.m10 = cam_pose.x*cam_pose.y - cam_pose.z*cam_pose.w;
  cached_odom->R.m11 = pow(cam_pose.y, 2) + pow(cam_pose.w, 2) - 0.5;
  cached_odom->R.m12 = cam_pose.y*cam_pose.z + cam_pose.x*cam_pose.w;
  cached_odom->R.m20 = cam_pose.x*cam_pose.z + cam_pose.y*cam_pose.w;
  cached_odom->R.m21 = cam_pose.y*cam_pose.z - cam_pose.x*cam_pose.w;
  cached_odom->R.m22 = pow(cam_pose.w, 2) + pow(cam_pose.z, 2) - 0.5;

  // Non-JPL Convention quaternion --> ORUtils Matrix
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

void ROSOdometrySourceEngine::VisualizePose() {

  // Construct pose
  cv::Mat pose_mat(3, 3, CV_32F);
  float* mat_pointer = (float*)pose_mat.data;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      mat_pointer[3*row + col] = cached_odom->R(col, row);
    }
  }
  viz_cam_pose.rotation(pose_mat);
  viz_cam_pose.translate(cv::Vec3f(0.0, 0.0, 0.0));
  viz_window.setWidgetPose("Camera Widget", viz_cam_pose);
  viz_window.spinOnce(1, true);
}

bool ROSOdometrySourceEngine::hasMoreMeasurements(void)
{
  return (cached_odom != NULL);
}

void ROSOdometrySourceEngine::getMeasurement(ITMOdometryMeasurement *odom)
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
