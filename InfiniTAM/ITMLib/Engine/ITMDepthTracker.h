// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMImageHierarchy.h"
#include "../Objects/ITMTemplatedHierarchyLevel.h"
#include "../Objects/ITMSceneHierarchyLevel.h"

#include "../Engine/ITMTracker.h"
#include "../Engine/ITMLowLevelEngine.h"

#include "nabo/nabo.h"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>


using namespace ITMLib::Objects;

namespace ITMLib
{
	namespace Engine
	{
		/** Base class for engine performing ICP based depth tracking.
		    A typical example would be the original KinectFusion
		    tracking algorithm.
		*/
		class ITMDepthTracker : public ITMTracker
		{
		private:
			const ITMLowLevelEngine *lowLevelEngine;
			ITMImageHierarchy<ITMSceneHierarchyLevel> *sceneHierarchy;
			ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> > *viewHierarchy;

			ITMTrackingState *trackingState; const ITMView *view;

			int *noIterationsPerLevel;
			int noICPLevel;

			float terminationThreshold;

			// PCL viewer
			bool pcl_render_stop = false;
			pcl::visualization::PCLVisualizer pc_viewer;
			pcl::PointCloud<pcl::PointXYZRGB> scene_cloud, current_view_cloud;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_pointer, current_view_cloud_pointer;
			void pcl_render_loop();

			void PrepareForEvaluation();
			void SetEvaluationParams(int levelId);

			void ComputeDelta(float *delta, float *nabla, float *hessian, bool shortIteration) const;
			void ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const;
			bool HasConverged(float *step) const;

			void SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view);

			const Eigen::MatrixXf ITMVectorToEigenMatrix(const Vector4f* vector, const Vector2i dim);
		protected:
			float *distThresh;

			int levelId;
			TrackerIterationType iterationType;

			Matrix4f scenePose;
			ITMSceneHierarchyLevel *sceneHierarchyLevel;
			ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel;

			int memory_type;
			// libnabo kd-tree
//      Nabo::NNSearchF* nns;

			virtual int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) = 0;

			// 3D point vector to PCL point cloud
			const void Float4ImagetoPclPointCloud(
			    const ITMFloat4Image* im, pcl::PointCloud<pcl::PointXYZRGB>& cloud, Vector3i color,
			    int memory_type);

			// Depth Map to PCL point cloud
			const void FloatImagetoPclPointCloud(
			    const ITMFloatImage* im, pcl::PointCloud<pcl::PointXYZRGB>& cloud,
			    const Vector4f intrinsics, Vector3i color, int memory_type);

			// Tracker Visualization
			const void visualizeTracker(const ITMFloat4Image* scene, const ITMFloatImage* current_view,
			                            const Vector4f intrinsics, int memory_type);

		public:
			void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

			// libnabo kd-tree
      boost::shared_ptr<Nabo::NNSearchF> nns;

			ITMDepthTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				float terminationThreshold, const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType);
			virtual ~ITMDepthTracker(void);


		};
	}
}
