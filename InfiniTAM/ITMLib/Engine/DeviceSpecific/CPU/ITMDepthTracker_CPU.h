// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMDepthTracker.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMDepthTracker_CPU : public ITMDepthTracker
		{
		protected:
		  std::pair<Vector4f*, int> ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

			template<bool shortIteration, bool rotationOnly>
			Vector4f computePerPointGH_Depth_NN(THREADPTR(float) *localNabla, THREADPTR(float) *localHessian, THREADPTR(float) &localF,
			  const THREADPTR(int) & x, const THREADPTR(int) & y,
			  const CONSTPTR(float) &depth, const CONSTPTR(Vector2i) & viewImageSize, const CONSTPTR(Vector4f) & viewIntrinsics, const CONSTPTR(Vector2i) & sceneImageSize,
			  const CONSTPTR(Vector4f) & sceneIntrinsics, const CONSTPTR(Matrix4f) & approxInvPose, const CONSTPTR(Matrix4f) & scenePose, const CONSTPTR(Vector4f) *pointsMap,
			  const CONSTPTR(Vector4f) *normalsMap, float distThresh, const boost::shared_ptr<Nabo::NNSearchF>& nns);

			template<bool shortIteration, bool rotationOnly>
			Vector4f computePerPointGH_Depth_Ab_NN(THREADPTR(float) *A, THREADPTR(float) &b,
			  const THREADPTR(int) & x, const THREADPTR(int) & y,
			  const CONSTPTR(float) &depth, const CONSTPTR(Vector2i) & viewImageSize, const CONSTPTR(Vector4f) & viewIntrinsics, const CONSTPTR(Vector2i) & sceneImageSize,
			  const CONSTPTR(Vector4f) & sceneIntrinsics, const CONSTPTR(Matrix4f) & approxInvPose, const CONSTPTR(Matrix4f) & scenePose, const CONSTPTR(Vector4f) *pointsMap,
			  const CONSTPTR(Vector4f) *normalsMap, float distThresh, const boost::shared_ptr<Nabo::NNSearchF>& nns);

		public:
			ITMDepthTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				float terminationThreshold, bool visualize_icp, const ITMLowLevelEngine *lowLevelEngine);
			~ITMDepthTracker_CPU(void);
		};
	}
}
