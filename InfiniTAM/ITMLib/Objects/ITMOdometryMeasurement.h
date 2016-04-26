/*
 * ITMOdometryMeasurement.h
 *
 *  Created on: Feb 19, 2016
 *      Author: anurag
 */

#ifndef INFINITAM_INFINITAM_ITMLIB_OBJECTS_ITMODOMETRYMEASUREMENT_H_
#define INFINITAM_INFINITAM_ITMLIB_OBJECTS_ITMODOMETRYMEASUREMENT_H_

#pragma once

#include "../Utils/ITMLibDefines.h"

namespace ITMLib
{
  namespace Objects
  {
    class ITMOdometryMeasurement
    {
    public:
      Matrix3f R;
      Vector3f t;
      Matrix6d cov;

      ITMOdometryMeasurement()
      {
        this->R.setIdentity();
        this->t.x = 0.0; this->t.y = 0.0; this->t.z = 0.0;
        this->cov.setIdentity();
      }

      ITMOdometryMeasurement(const Matrix3f & R, const Vector3f t)
      {
        this->R = R;
        this->t = t;
      }

      ITMOdometryMeasurement(const Matrix3f & R, const Vector3f t, const Matrix6d cov)
      {
        this->R = R;
        this->t = t;
        this->cov = cov;
      }

      ITMOdometryMeasurement(const Matrix4f & T)
      {
        this->R = T.getRot();
        this->t = T.getTrans();
      }

      ITMOdometryMeasurement(const Matrix4f & T, const Matrix6d cov)
      {
        this->R = T.getRot();
        this->t = T.getTrans();
        this->cov = cov;
      }

      void SetFrom(const ITMOdometryMeasurement *measurement)
      {
        this->R = measurement->R;
        this->t = measurement->t;
      }

      Matrix4f GetM() {
        Matrix4f T(R.m00, R.m01, R.m02, 0.0,
                   R.m10, R.m11, R.m12, 0.0,
                   R.m20, R.m21, R.m22, 0.0,
                   t.x, t.y, t.z, 1.0);
        return T;
      }

      ~ITMOdometryMeasurement(void) { }

      // Suppress the default copy constructor and assignment operator
      ITMOdometryMeasurement(const ITMOdometryMeasurement&);
      ITMOdometryMeasurement& operator=(const ITMOdometryMeasurement&);
    };
  }
}

#endif /* INFINITAM_INFINITAM_ITMLIB_OBJECTS_ITMODOMETRYMEASUREMENT_H_ */
