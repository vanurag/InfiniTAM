// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMDepthTracker.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMDepthTracker_CUDA : public ITMDepthTracker
		{
		public:
			struct AccuCell;

		private:
			AccuCell *accu_host;
			AccuCell *accu_device;

		protected:
			std::pair<Vector4f*, int> ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);
			Matrix4f getLPMICPTF(Matrix4f& prevInvPose) {};

		public:
			ITMDepthTracker_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				float terminationThreshold, ITMLibSettings::DepthTrackerType tracker_type, bool visualize_icp, const ITMLowLevelEngine *lowLevelEngine);
			~ITMDepthTracker_CUDA(void);
		};
	}
}
