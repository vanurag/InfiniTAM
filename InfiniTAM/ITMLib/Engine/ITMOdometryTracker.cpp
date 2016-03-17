/*
 * ITMOdometryTracker.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: anurag
 */

#include "ITMOdometryTracker.h"
#include "../Objects/ITMViewOdometry.h"

using namespace ITMLib::Engine;

ITMOdometryTracker::ITMOdometryTracker(ITMIMUCalibrator *calibrator)
{
  this->calibrator = calibrator;
}

ITMOdometryTracker::~ITMOdometryTracker(void)
{
}

void ITMOdometryTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
  std::cout << "Using ODOM Tracker!!!!!!!!!!!" << std::endl;
  calibrator->RegisterMeasurement(((ITMViewOdometry*)view)->odom->GetM());

  Matrix4f T_rgb_to_depth = ((ITMViewOdometry*)view)->calib->trafo_rgb_to_depth.calib;
  Matrix4f T_depth_to_rgb;
  T_rgb_to_depth.inv(T_depth_to_rgb);
  trackingState->pose_d->SetM(
      T_rgb_to_depth *
      calibrator->GetDifferentialTrafoChange() *
      T_depth_to_rgb *
      trackingState->pose_d->GetM());
  trackingState->pose_d->Coerce();
}
