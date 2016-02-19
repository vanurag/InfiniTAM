// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMIMUTracker.h"
#include "../Objects/ITMViewIMU.h"

using namespace ITMLib::Engine;

ITMIMUTracker::ITMIMUTracker(ITMIMUCalibrator *calibrator)
{
	this->calibrator = calibrator;
}

ITMIMUTracker::~ITMIMUTracker(void)
{
}

void ITMIMUTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
  std::cout << "Using IMU Tracker!!!!!!!!!!!" << std::endl;
	calibrator->RegisterMeasurement(((ITMViewIMU*)view)->imu->R);

	Matrix4f T = ((ITMViewIMU*)view)->calib->trafo_rgb_to_depth.calib;
	Matrix3f R_rgb_to_depth(T.m00, T.m01, T.m02, T.m10, T.m11, T.m12, T.m20, T.m21, T.m22);
	Matrix3f R_depth_to_rgb(T.m00, T.m10, T.m20, T.m01, T.m11, T.m21, T.m02, T.m12, T.m22);
	trackingState->pose_d->SetR(
	    R_rgb_to_depth *
	    calibrator->GetDifferentialRotationChange() *
	    R_depth_to_rgb *
      trackingState->pose_d->GetR());
}
