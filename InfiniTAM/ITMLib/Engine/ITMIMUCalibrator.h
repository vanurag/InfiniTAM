// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../Objects/ITMPose.h"

namespace ITMLib
{
	namespace Objects
	{
		class ITMIMUCalibrator
		{
		public:
			virtual void RegisterMeasurement(const Matrix3f & R) = 0;
			virtual Matrix3f GetDifferentialRotationChange() = 0;

			ITMIMUCalibrator() { }

			virtual ~ITMIMUCalibrator(void) { }

			// Suppress the default copy constructor and assignment operator
			ITMIMUCalibrator(const ITMIMUCalibrator&);
			ITMIMUCalibrator& operator=(const ITMIMUCalibrator&);
		};

		class ITMIMUCalibrator_iPad : public ITMIMUCalibrator
		{
		private:
			ITMPose *imuPose_imucoords, *imuPose_cameracoords;
			Vector3f t_imu, r_imu;
			Matrix3f inv_oldR_imu;
			Matrix3f newR_imu, oldR_imu;
			bool hasTwoFrames;

		public: 
			void RegisterMeasurement(const Matrix3f & R)
			{
				oldR_imu = imuPose_imucoords->GetR();

				imuPose_imucoords->SetR(R);

				imuPose_imucoords->GetParams(t_imu, r_imu);
				imuPose_imucoords->SetFrom(t_imu, -r_imu);

				newR_imu = imuPose_imucoords->GetR();
			}

			Matrix3f GetDifferentialRotationChange()
			{
				if (hasTwoFrames)
				{
					oldR_imu.inv(inv_oldR_imu);
					imuPose_cameracoords->SetR(imuPose_imucoords->GetR() * inv_oldR_imu);

					imuPose_cameracoords->GetParams(t_imu, r_imu);
					imuPose_cameracoords->SetFrom(t_imu.x, t_imu.y, t_imu.z, -r_imu.y, -r_imu.x, -r_imu.z);
				}
				
				hasTwoFrames = true;
				return imuPose_cameracoords->GetR();
			}

			ITMIMUCalibrator_iPad() : ITMIMUCalibrator() 
			{ 
				hasTwoFrames = false;

				imuPose_imucoords = new ITMPose();
				imuPose_imucoords->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

				imuPose_cameracoords = new ITMPose();
				imuPose_cameracoords->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

				oldR_imu.setIdentity();
			}

			~ITMIMUCalibrator_iPad(void) 
			{
				delete imuPose_imucoords;
				delete imuPose_cameracoords;
			}
		};

		class ITMIMUCalibrator_DRZ : public ITMIMUCalibrator
    {
    private:
      Matrix3f differential_rotation_change;
      Vector3f t_imu, r_imu;
      Matrix3f inv_oldR_imu;
      Matrix3f newR_imu, oldR_imu;
      bool hasTwoFrames;

    public:
      void RegisterMeasurement(const Matrix3f & R)
      {
        newR_imu = R;
      }

      Matrix3f GetDifferentialRotationChange()
      {
        if (hasTwoFrames)
        {
          oldR_imu.inv(inv_oldR_imu);
          differential_rotation_change = newR_imu * inv_oldR_imu;
        } else {
          differential_rotation_change.setIdentity();
        }
        hasTwoFrames = true;
        oldR_imu = newR_imu;
        return differential_rotation_change;
      }

      ITMIMUCalibrator_DRZ() : ITMIMUCalibrator()
      {
        hasTwoFrames = false;
        oldR_imu.setIdentity();
        newR_imu.setIdentity();
      }

      ~ITMIMUCalibrator_DRZ(void) {}
    };

		class ITMIMUCalibrator_DRZ2 : public ITMIMUCalibrator
    {
    private:
      ITMPose *imuPose_imucoords, *camPose_imucoords, *diffImuPose_cameracoords;
      Vector3f t_imu, r_imu;
      Matrix3f inv_oldR_imu;
      Matrix3f newR_imu, oldR_imu;
      bool hasTwoFrames;

    public:
      void RegisterMeasurement(const Matrix3f & R)
      {
        oldR_imu = imuPose_imucoords->GetR();

        imuPose_imucoords->SetR(R);

        Matrix3f T_cam_imu(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
        camPose_imucoords->SetR(T_cam_imu*R);
      }

      Matrix3f GetDifferentialRotationChange()
      {
        if (hasTwoFrames)
        {
          oldR_imu.inv(inv_oldR_imu);
          Matrix3f T_imu_cam(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
          diffImuPose_cameracoords->SetR(camPose_imucoords->GetR() * inv_oldR_imu * T_imu_cam);
        }

        hasTwoFrames = true;
        return diffImuPose_cameracoords->GetR();
      }

      ITMIMUCalibrator_DRZ2() : ITMIMUCalibrator()
      {
        hasTwoFrames = false;

        imuPose_imucoords = new ITMPose();
        imuPose_imucoords->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        diffImuPose_cameracoords = new ITMPose();
        diffImuPose_cameracoords->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        camPose_imucoords = new ITMPose();
        camPose_imucoords->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        oldR_imu.setIdentity();
      }

      ~ITMIMUCalibrator_DRZ2(void)
      {
        delete imuPose_imucoords;
        delete camPose_imucoords;
        delete diffImuPose_cameracoords;
      }
    };
	}
}

