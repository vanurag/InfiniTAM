// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../Objects/ITMPose.h"
#include <limits>

namespace ITMLib
{
	namespace Objects
	{
		class ITMIMUCalibrator
		{
		public:
			virtual void RegisterMeasurement(const Matrix3f & R) = 0;
			virtual void RegisterMeasurement(const Matrix4f & T) = 0;
			virtual Matrix3f GetDifferentialRotationChange() = 0;
			virtual Matrix4f GetDifferentialTrafoChange() = 0;

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

			void RegisterMeasurement(const Matrix4f & T) {
			  std::cout << "Odometry measurements not supported for ITMIMUCalibrator_iPad" << std::endl;
			  exit(1);
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

			Matrix4f GetDifferentialTrafoChange() {
			  std::cout << "Odometry measurements not supported for ITMIMUCalibrator_iPad" << std::endl;
        exit(1);
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

		// Kinect - VI-sensor ROVIO RIG
    class ITMIMUCalibrator_DRZ : public ITMIMUCalibrator
    {
    private:
		  ITMPose *imuPose_imucoords, *camPose_imucoords, *diffImuPose_cameracoords;
      Matrix4f T_rgb_imu, T_imu_rgb;
      Matrix3f R_rgb_imu, R_imu_rgb;
      Vector3f t_rgb_imu, t_imu_rgb;
      Vector3f t_imu, r_imu;
      Matrix3f inv_oldR_imu;
      Matrix4f inv_oldT_imu;
      Matrix3f newR_imu, oldR_imu;
      Matrix4f newT_imu, oldT_imu;
      bool hasTwoFrames;

    public:
      void RegisterMeasurement(const Matrix3f & R)
      {
        oldR_imu = imuPose_imucoords->GetR();
        imuPose_imucoords->SetR(R);
        camPose_imucoords->SetR(R_rgb_imu * R);
      }

      void RegisterMeasurement(const Matrix4f & T) {
        oldT_imu = imuPose_imucoords->GetM();
        oldR_imu = oldT_imu.getRot();
        imuPose_imucoords->SetM(T);
        camPose_imucoords->SetM(T_rgb_imu * T);
      }

      Matrix3f GetDifferentialRotationChange()
      {
        if (hasTwoFrames)
        {
          oldR_imu.inv(inv_oldR_imu);
          diffImuPose_cameracoords->SetR(camPose_imucoords->GetR() * inv_oldR_imu * R_imu_rgb);
        }

        hasTwoFrames = true;
        return diffImuPose_cameracoords->GetR();
      }

      Matrix4f GetDifferentialTrafoChange()
      {
        if (hasTwoFrames)
        {
          oldT_imu.inv(inv_oldT_imu);
          diffImuPose_cameracoords->SetM(camPose_imucoords->GetM() * inv_oldT_imu * T_imu_rgb);
        }

        hasTwoFrames = true;
        return diffImuPose_cameracoords->GetM();
      }

      ITMIMUCalibrator_DRZ() : ITMIMUCalibrator(),
          T_rgb_imu(-1.0, 0.0, 0.0, 0.0,     // (TODO) needs to be calibrated
                    0.0, -1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0),
          T_imu_rgb(-1.0, 0.0, 0.0, 0.0,
                    0.0, -1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0)
      {
        hasTwoFrames = false;

        // checks
        R_rgb_imu = T_rgb_imu.getRot(); R_imu_rgb = T_imu_rgb.getRot();
        t_rgb_imu = T_rgb_imu.getTrans(); t_imu_rgb = T_imu_rgb.getTrans();
        Vector3f temp = t_rgb_imu + R_rgb_imu*t_imu_rgb;  // should be all zeros ideally
        if (R_rgb_imu.t() != R_imu_rgb ||
            abs(temp.x) + abs(temp.y) + abs(temp.z) >
                3*std::numeric_limits<float>::epsilon()) {
          std::cout << "IMU-Cam Calibration matrices not consistent" << std::endl;
          std::cout << "check 1: " << (R_rgb_imu.t() != R_imu_rgb) << std::endl;
          std::cout << "check 2: " << abs(temp.x) + abs(temp.y) + abs(temp.z) << std::endl;
          exit(1);
        }

        imuPose_imucoords = new ITMPose();
        imuPose_imucoords->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        diffImuPose_cameracoords = new ITMPose();
        diffImuPose_cameracoords->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        camPose_imucoords = new ITMPose();
        camPose_imucoords->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        oldR_imu.setIdentity();
        oldT_imu.setIdentity();
      }

      ~ITMIMUCalibrator_DRZ(void)
      {
        delete imuPose_imucoords;
        delete camPose_imucoords;
        delete diffImuPose_cameracoords;
      }
    };

		// Realsense - VI-sensor ROVIO RIG
		class ITMIMUCalibrator_DRZ2 : public ITMIMUCalibrator
    {
    private:
      ITMPose *imuPose_imucoords, *camPose_imucoords, *diffImuPose_cameracoords;
      Matrix4f T_rgb_imu, T_imu_rgb;
      Matrix3f R_rgb_imu, R_imu_rgb;
      Vector3f t_rgb_imu, t_imu_rgb;
      Vector3f t_imu, r_imu;
      Matrix3f inv_oldR_imu;
      Matrix4f inv_oldT_imu;
      Matrix3f newR_imu, oldR_imu;
      Matrix4f newT_imu, oldT_imu;
      bool hasTwoFrames;

    public:
      void RegisterMeasurement(const Matrix3f & R)
      {
        oldR_imu = imuPose_imucoords->GetR();
        imuPose_imucoords->SetR(R);
        camPose_imucoords->SetR(R_rgb_imu * R);
      }

      void RegisterMeasurement(const Matrix4f & T) {
        oldT_imu = imuPose_imucoords->GetM();
        oldR_imu = oldT_imu.getRot();
        imuPose_imucoords->SetM(T);
        camPose_imucoords->SetM(T_rgb_imu * T);
      }

      Matrix3f GetDifferentialRotationChange()
      {
        if (hasTwoFrames)
        {
          oldR_imu.inv(inv_oldR_imu);
          diffImuPose_cameracoords->SetR(camPose_imucoords->GetR() * inv_oldR_imu * R_imu_rgb);
        }

        hasTwoFrames = true;
        return diffImuPose_cameracoords->GetR();
      }

      Matrix4f GetDifferentialTrafoChange()
      {
        if (hasTwoFrames)
        {
          oldT_imu.inv(inv_oldT_imu);
          diffImuPose_cameracoords->SetM(camPose_imucoords->GetM() * inv_oldT_imu * T_imu_rgb);
        }

        hasTwoFrames = true;
        return diffImuPose_cameracoords->GetM();
      }

      ITMIMUCalibrator_DRZ2() : ITMIMUCalibrator(),
          T_rgb_imu(-0.99485704, -0.04357693, 0.09143589, 0.0,
                    0.05709121, -0.98691011, 0.1508278, 0.0,
                    0.08366639, 0.15527229, 0.98432233, 0.0,
                    0.10094737, -0.06895541, -0.48799559, 1.0),
          T_imu_rgb(-0.99485704, 0.05709121, 0.08366639, 0.0,
                    -0.04357693, -0.98691011, 0.15527229, 0.0,
                    0.09143589, 0.1508278, 0.98432233, 0.0,
                    0.14204364, -0.00021269, 0.48260592, 1.0)
      {
        hasTwoFrames = false;

        // checks
        R_rgb_imu = T_rgb_imu.getRot(); R_imu_rgb = T_imu_rgb.getRot();
        t_rgb_imu = T_rgb_imu.getTrans(); t_imu_rgb = T_imu_rgb.getTrans();
        Vector3f temp = t_rgb_imu + R_rgb_imu*t_imu_rgb;  // should be all zeros ideally
        if (R_rgb_imu.t() != R_imu_rgb ||
            abs(temp.x) + abs(temp.y) + abs(temp.z) >
                3*std::numeric_limits<float>::epsilon()) {
          std::cout << "IMU-Cam Calibration matrices not consistent" << std::endl;
          std::cout << "check 1: " << (R_rgb_imu.t() != R_imu_rgb) << std::endl;
          std::cout << "check 2: " << abs(temp.x) + abs(temp.y) + abs(temp.z) << std::endl;
          exit(1);
        }

        imuPose_imucoords = new ITMPose();
        imuPose_imucoords->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        diffImuPose_cameracoords = new ITMPose();
        diffImuPose_cameracoords->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        camPose_imucoords = new ITMPose();
        camPose_imucoords->SetFrom(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        oldR_imu.setIdentity();
        oldT_imu.setIdentity();
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

