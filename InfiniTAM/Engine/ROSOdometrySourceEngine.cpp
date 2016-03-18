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
                                    &ROSOdometrySourceEngine::ROSOdometryCallback_Odom, this);
        break;
      }
      if (topic.datatype == std::string("geometry_msgs/TransformStamped")) {
        sub_pose_ = node_.subscribe(node_.resolveName(odomMask), 1,
                                    &ROSOdometrySourceEngine::ROSTFCallback_Odom, this);
        break;
      }
      if (topic.datatype == std::string("geometry_msgs/PoseStamped")) {
        sub_pose_ = node_.subscribe(node_.resolveName(odomMask), 1,
                                    &ROSOdometrySourceEngine::ROSPoseCallback_Odom, this);
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

ROSOdometrySourceEngine::ROSOdometrySourceEngine() : OdometrySourceEngine("")
{
  cached_odom = NULL;
}

void ROSOdometrySourceEngine::ROSOdometryCallback_Odom(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Odometry Orientation x: [%f], y: [%f], z: [%f], w: [%f]",
           msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
           msg->pose.pose.orientation.w);
  quat2ITMOdom(Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
  Vector3f t_imu_G = cached_odom->R * Vector3f(-msg->pose.pose.position.x,
                                               -msg->pose.pose.position.y,
                                               -msg->pose.pose.position.z);
  cached_odom->t.x = t_imu_G.x;
  cached_odom->t.y = t_imu_G.y;
  cached_odom->t.z = t_imu_G.z;
}

void ROSOdometrySourceEngine::ROSTFCallback_Odom(
    const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  ROS_INFO("TF Orientation x: [%f], y: [%f], z: [%f], w: [%f]",
           msg->transform.rotation.x, msg->transform.rotation.y,
           msg->transform.rotation.z, msg->transform.rotation.w);
  quat2ITMOdom(Quaternion(msg->transform.rotation.x, msg->transform.rotation.y,
                          msg->transform.rotation.z, msg->transform.rotation.w));
  Vector3f t_imu_G = cached_odom->R * Vector3f(-msg->transform.translation.x,
                                               -msg->transform.translation.y,
                                               -msg->transform.translation.z);
  cached_odom->t.x = t_imu_G.x;
  cached_odom->t.y = t_imu_G.y;
  cached_odom->t.z = t_imu_G.z;
}

void ROSOdometrySourceEngine::ROSPoseCallback_Odom(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO("Pose Orientation x: [%f], y: [%f], z: [%f], w: [%f]",
           msg->pose.orientation.x, msg->pose.orientation.y,
           msg->pose.orientation.z, msg->pose.orientation.w);
  quat2ITMOdom(Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                          msg->pose.orientation.z, msg->pose.orientation.w));
  Vector3f t_imu_G = cached_odom->R * Vector3f(-msg->pose.position.x,
                                               -msg->pose.position.y,
                                               -msg->pose.position.z);
  cached_odom->t.x = t_imu_G.x;
  cached_odom->t.y = t_imu_G.y;
  cached_odom->t.z = t_imu_G.z;
}

// conversion from quaternion to rotation matrix
void ROSOdometrySourceEngine::quat2ITMOdom(const Quaternion odom_pose) {

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
  cached_odom->R.m00 = 2*pow(cam_pose.w, 2) + 2*pow(cam_pose.x, 2) - 1.0;
  cached_odom->R.m01 = 2*cam_pose.x*cam_pose.y + 2*cam_pose.z*cam_pose.w;
  cached_odom->R.m02 = 2*cam_pose.x*cam_pose.z - 2*cam_pose.y*cam_pose.w;
  cached_odom->R.m10 = 2*cam_pose.x*cam_pose.y - 2*cam_pose.z*cam_pose.w;
  cached_odom->R.m11 = 2*pow(cam_pose.y, 2) + 2*pow(cam_pose.w, 2) - 1.0;
  cached_odom->R.m12 = 2*cam_pose.y*cam_pose.z + 2*cam_pose.x*cam_pose.w;
  cached_odom->R.m20 = 2*cam_pose.x*cam_pose.z + 2*cam_pose.y*cam_pose.w;
  cached_odom->R.m21 = 2*cam_pose.y*cam_pose.z - 2*cam_pose.x*cam_pose.w;
  cached_odom->R.m22 = 2*pow(cam_pose.w, 2) + 2*pow(cam_pose.z, 2) - 1.0;

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

//  VisualizePose();

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

