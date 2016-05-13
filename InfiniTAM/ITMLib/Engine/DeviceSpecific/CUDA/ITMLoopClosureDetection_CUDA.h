/*
 * ITMLoopClosureDetection_CUDA.h
 *
 *  Created on: May 13, 2016
 *      Author: anurag
 */

// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#ifndef ITMLOOPCLOSUREDETECTION_CUDA_H_
#define ITMLOOPCLOSUREDETECTION_CUDA_H_

#pragma once

#include "../../ITMLoopClosureDetection.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMLoopClosureDetection_CUDA : public ITMLoopClosureDetection
		{
		public:
			struct AccuCell;

		private:
			AccuCell *accu_host;
			AccuCell *accu_device;

		protected:
			std::pair<Vector4f*, int> ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

		public:
			ITMLoopClosureDetection_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				float terminationThreshold, ITMLibSettings::DepthTrackerType tracker_type, bool visualize_icp, const ITMLowLevelEngine *lowLevelEngine);
			~ITMLoopClosureDetection_CUDA(void);
		};
	}
}

#endif /* ITMLOOPCLOSUREDETECTION_CUDA_H_ */
