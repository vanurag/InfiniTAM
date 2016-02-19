/*
 * OdometrySourceEngine.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: anurag
 */

#include "OdometrySourceEngine.h"

#include "../Utils/FileUtils.h"

#include <stdio.h>

using namespace InfiniTAM::Engine;

OdometrySourceEngine::OdometrySourceEngine(const char *imuMask)
{
  strncpy(this->odomMask, imuMask, BUF_SIZE);

  currentFrameNo = 0;
  cachedFrameNo = -1;

  cached_odom = NULL;
}

void OdometrySourceEngine::loadOdometryIntoCache(void)
{
  char str[2048]; FILE *f; bool success = false;

  cached_odom = new ITMOdometryMeasurement();

  sprintf(str, odomMask, currentFrameNo);
  f = fopen(str, "r");
  if (f)
  {
    size_t ret = fscanf(f, "%f %f %f %f %f %f %f %f %f %f %f %f",
      &cached_odom->R.m00, &cached_odom->R.m01, &cached_odom->R.m02,
      &cached_odom->R.m10, &cached_odom->R.m11, &cached_odom->R.m12,
      &cached_odom->R.m20, &cached_odom->R.m21, &cached_odom->R.m22,
      &cached_odom->t.x, &cached_odom->t.y, &cached_odom->t.z);

    fclose(f);

    if (ret == 12) success = true;
  }

  if (!success) {
    delete cached_odom; cached_odom = NULL;
    printf("error reading file '%s'\n", str);
  }
}

bool OdometrySourceEngine::hasMoreMeasurements(void)
{
  loadOdometryIntoCache();

  return (cached_odom != NULL);
}

void OdometrySourceEngine::getMeasurement(ITMOdometryMeasurement *odom)
{
  bool bUsedCache = false;

  if (cached_odom != NULL)
  {
    odom->R = cached_odom->R;
    odom->t = cached_odom->t;
    delete cached_odom;
    cached_odom = NULL;
    bUsedCache = true;
  }

  if (!bUsedCache) this->loadOdometryIntoCache();

  ++currentFrameNo;
}


