/*
 * OdometrySourceEngine.h
 *
 *  Created on: Feb 19, 2016
 *      Author: anurag
 */

#ifndef INFINITAM_INFINITAM_ENGINE_ODOMETRYSOURCEENGINE_H_
#define INFINITAM_INFINITAM_ENGINE_ODOMETRYSOURCEENGINE_H_

#pragma once

#include "../ITMLib/ITMLib.h"

namespace InfiniTAM
{
  namespace Engine
  {
    class OdometrySourceEngine
    {
    private:
      static const int BUF_SIZE = 2048;
      char odomMask[BUF_SIZE];

      ITMOdometryMeasurement *cached_odom;

      void loadOdometryIntoCache();
      int cachedFrameNo;
      int currentFrameNo;

    public:
      OdometrySourceEngine(const char *odomMask);
      virtual ~OdometrySourceEngine() { }

      virtual bool hasMoreMeasurements(void);
      virtual void getMeasurement(ITMOdometryMeasurement *odom);
    };
  }
}

#endif /* INFINITAM_INFINITAM_ENGINE_ODOMETRYSOURCEENGINE_H_ */
