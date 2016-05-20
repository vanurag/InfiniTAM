// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "../Objects/ITMImageHierarchy.h"
#include "../Objects/ITMTemplatedHierarchyLevel.h"
#include "../Objects/ITMSceneHierarchyLevel.h"

#include "../Utils/ITMLibSettings.h"

#include "../Engine/ITMTracker.h"
#include "../Engine/ITMLowLevelEngine.h"

// Libpointmatcher
#include "pointmatcher/PointMatcher.h"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>

#include "../../Utils/gnuplot-iostream/gnuplot-iostream.h"


using namespace ITMLib::Objects;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

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
			bool viz_icp = false; // Whether or not to run visualization routine
			bool pcl_render_stop = false;
			pcl::visualization::PCLVisualizer pc_viewer;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud_pointer, current_view_cloud_pointer;
			void pcl_render_loop();
			//GNU Plot
			Gnuplot gp;
			std::vector<double> gp_outlier_dist;

			void PrepareForEvaluation();
			void SetEvaluationParams(int levelId);

			void ComputeDelta(float *delta, float *nabla, float *hessian, bool shortIteration) const;
			void ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const;
			bool HasConverged(float *step) const;

			void SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view);

			const Eigen::MatrixXf ITMVectorToEigenMatrix(const Vector4f* vector, const Vector2i dim);
		protected:
			float *distThresh;

			// tracker type
			ITMLibSettings::DepthTrackerType type;

			int levelId;
			TrackerIterationType iterationType;

			Matrix4f scenePose, sceneInvPose;
			ITMSceneHierarchyLevel *sceneHierarchyLevel;
			ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel;

			int memory_type;
			// libnabo kd-tree
//      Nabo::NNSearchF* nns;

			virtual std::pair<Vector4f*, int> ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) = 0;
			// Use libpointmatcher for ICP routine
      virtual Matrix4f getLPMICPTF(Matrix4f& prevInvPose) = 0;

			// 3D point vector to PCL point cloud
			void Float4ImagetoPclPointCloud(
			    const ITMFloat4Image* im, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Vector3i color,
			    int memory_type);

			// Depth Map to PCL point cloud
			void FloatImagetoPclPointCloud(
			    const ITMFloatImage* im, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
			    const Vector4f intrinsics, Vector3i color, int memory_type, std::vector<Matrix4f*>& tf_chain);

			// Draw ICP point matches
			void DrawPointMatches(
			    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Vector4f* matches, Vector3i color);

			// Tracker Matches Visualization
			void visualizeTracker(
			    const ITMFloat4Image* scene, const ITMFloatImage* current_view,
			    const Vector4f intrinsics, Vector4f* matches, int memory_type, std::vector<Matrix4f*>& tf_chain);

			// Tracker TF Update Visualization
			void visualizeTracker(
			    const ITMFloat4Image* scene, const ITMFloatImage* current_view, const Vector4f intrinsics,
			    int memory_type, std::vector<Matrix4f*>& tf_chain, bool converged);

		public:
			void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);

			// libnabo kd-tree
      boost::shared_ptr<Nabo::NNSearchF> nns;

			ITMDepthTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
				float terminationThreshold, ITMLibSettings::DepthTrackerType tracker_type, bool visualize_icp, const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType);
			virtual ~ITMDepthTracker(void);


		};
	}
}
