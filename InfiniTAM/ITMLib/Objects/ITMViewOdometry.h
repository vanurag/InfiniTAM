/*
 * ITMViewOdometry.h
 *
 *  Created on: Feb 19, 2016
 *      Author: anurag
 */

#ifndef INFINITAM_INFINITAM_ITMLIB_OBJECTS_ITMVIEWODOMETRY_H_
#define INFINITAM_INFINITAM_ITMLIB_OBJECTS_ITMVIEWODOMETRY_H_

#pragma once

#include "../Objects/ITMView.h"
#include "../Objects/ITMOdometryMeasurement.h"

namespace ITMLib
{
  namespace Objects
  {
    /** \brief
        Represents a single "view", i.e. RGB and depth images along
        with all intrinsic and relative calibration information
    */
    class ITMViewOdometry : public ITMView
    {
    public:
      ITMOdometryMeasurement *odom;

      ITMViewOdometry(const ITMRGBDCalib *calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU)
       : ITMView(calibration, imgSize_rgb, imgSize_d, useGPU)
      {
        odom = new ITMOdometryMeasurement();
      }

      ~ITMViewOdometry(void) { delete odom; }

      // Suppress the default copy constructor and assignment operator
      ITMViewOdometry(const ITMViewOdometry&);
      ITMViewOdometry& operator=(const ITMViewOdometry&);
    };
  }
}

#endif /* INFINITAM_INFINITAM_ITMLIB_OBJECTS_ITMVIEWODOMETRY_H_ */
