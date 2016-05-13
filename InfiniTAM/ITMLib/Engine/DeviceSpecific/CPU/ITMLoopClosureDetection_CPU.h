/*
 * ITMLoopClosureDetection_CPU.h
 *
 *  Created on: May 13, 2016
 *      Author: anurag
 */

// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#ifndef INFINITAM_INFINITAM_ITMLIB_ENGINE_DEVICESPECIFIC_CPU_ITMLOOPCLOSUREDETECTION_CPU_H_
#define INFINITAM_INFINITAM_ITMLIB_ENGINE_DEVICESPECIFIC_CPU_ITMLOOPCLOSUREDETECTION_CPU_H_

#pragma once

#include "../../ITMLoopClosureDetection.h"

namespace ITMLib
{
  namespace Engine
  {
    class ITMLoopClosureDetection_CPU : public ITMLoopClosureDetection
    {
    protected:
      std::pair<Vector4f*, int> ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

    public:
      ITMLoopClosureDetection_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
        float terminationThreshold, ITMLibSettings::DepthTrackerType tracker_type, bool visualize_icp, const ITMLowLevelEngine *lowLevelEngine);
      ~ITMLoopClosureDetection_CPU(void);
    };
  }
}


#endif /* INFINITAM_INFINITAM_ITMLIB_ENGINE_DEVICESPECIFIC_CPU_ITMLOOPCLOSUREDETECTION_CPU_H_ */
