// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Objects/ITMSceneParams.h"
//#include "../Engine/ITMTracker.h"

namespace ITMLib
{
	namespace Objects
	{
		class ITMLibSettings
		{
		public:
			/// The device used to run the DeviceAgnostic code
			typedef enum {
				DEVICE_CPU,
				DEVICE_CUDA,
				DEVICE_METAL
			} DeviceType;

			/// Select the type of device to use
			DeviceType deviceType;

			/// Enables swapping between host and device.
			bool useSwapping;

			bool useApproximateRaycast;

			bool useBilateralFilter;

			bool modelSensorNoise;

			/// Tracker types
			typedef enum {
			  // non-composite trackers
				//! Identifies a tracker based on colour image
				TRACKER_COLOR,
				//! Identifies a tracker based on depth image
				TRACKER_ICP,
				//! Identifies a tracker based on depth image (Ren et al, 2012)
				TRACKER_REN,
				//! Identifies a tracker based on Odometry measurement
        TRACKER_STRICT_ODOMETRY,
        //! Identifies a tracker that use weighted ICP only on depth image
        TRACKER_WICP,

        // ----------------------------------------------------------------

				// composite trackers
				//! Identifies a tracker based on depth image and IMU measurement
				TRACKER_IMU,
				//! Identifies a tracker based on depth image and Odometry measurement
        TRACKER_ODOMETRY,
        //! Identifies a tracker based on depth image, colour image and Odometry measurement
        TRACKER_ODOMETRY_COLOR
			} TrackerType;

			/// Depth Tracker types
      typedef enum {
        //! InfiniTAM default depth tracker on image hierarchy
        TRACKER_ITM,
        //! TRACKER_ITM but uses libnabo for point associations
        TRACKER_NABO,
        //! Using Libpointmatcher for ICP (only CPU)
        TRACKER_LPM,
        //! TRACKER_LPM but operates on image hierarchy (only CPU)
        TRACKER_LPM_HIERARCHY,
      } DepthTrackerType;

			/// Select the type of tracker to use
			TrackerType trackerType;
			DepthTrackerType depthTrackerType;

			/// The tracking regime used by the tracking controller
			TrackerIterationType *trackingRegime;

			/// The number of levels in the trackingRegime
			int noHierarchyLevels;
			
			/// Run ICP till # Hierarchy level, then switch to ITMRenTracker for local refinement.
			int noICPRunTillLevel;

			/// For ITMColorTracker: skip every other point in energy function evaluation.
			bool skipPoints;

			/// For ITMDepthTracker: ICP distance threshold
			float depthTrackerICPThreshold;

			/// For ITMDepthTracker: ICP iteration termination threshold
			float depthTrackerTerminationThreshold;

			// ICP visualization flag
			bool visualizeICP;

			/// Further, scene specific parameters such as voxel size
			ITMLib::Objects::ITMSceneParams sceneParams;

			ITMLibSettings(void);
			~ITMLibSettings(void);

			void setTrackerType(const TrackerType& new_tracker_type);

			// Suppress the default copy constructor and assignment operator
			ITMLibSettings(const ITMLibSettings&);
			ITMLibSettings& operator=(const ITMLibSettings&);
		};
	}
}
