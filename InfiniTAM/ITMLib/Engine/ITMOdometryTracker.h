/*
 * ITMOdometryTracker.h
 *
 *  Created on: Feb 19, 2016
 *      Author: anurag
 */

#ifndef INFINITAM_INFINITAM_ITMLIB_ENGINE_ITMODOMETRYTRACKER_H_
#define INFINITAM_INFINITAM_ITMLIB_ENGINE_ITMODOMETRYTRACKER_H_

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMOdometryMeasurement.h"

#include "../Engine/ITMTracker.h"
#include "../Engine/ITMLowLevelEngine.h"
#include "../Engine/ITMIMUCalibrator.h"

using namespace ITMLib::Objects;

namespace ITMLib
{
  namespace Engine
  {
    class ITMOdometryTracker : public ITMTracker
    {
    private:
      ITMIMUCalibrator *calibrator;

    public:
      void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

      ITMOdometryTracker(ITMIMUCalibrator *calibrator);
      virtual ~ITMOdometryTracker(void);
    };
  }
}

#endif /* INFINITAM_INFINITAM_ITMLIB_ENGINE_ITMODOMETRYTRACKER_H_ */
