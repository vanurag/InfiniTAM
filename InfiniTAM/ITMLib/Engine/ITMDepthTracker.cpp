// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthTracker.h"
#include "../../ORUtils/Cholesky.h"
#include "../Objects/ITMViewOdometry.h"

#include <math.h>

using namespace ITMLib::Engine;

ITMDepthTracker::ITMDepthTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel, float distThresh,
	float terminationThreshold, ITMLibSettings::DepthTrackerType tracker_type, bool visualize_icp, const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType)
    : pc_viewer("ICP visualizer"), scene_cloud_pointer(&scene_cloud),
      current_view_cloud_pointer(&current_view_cloud)
{
	viewHierarchy = new ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> >(imgSize, trackingRegime, noHierarchyLevels, memoryType, true);
	sceneHierarchy = new ITMImageHierarchy<ITMSceneHierarchyLevel>(imgSize, trackingRegime, noHierarchyLevels, memoryType, true);

	this->noIterationsPerLevel = new int[noHierarchyLevels];
	this->distThresh = new float[noHierarchyLevels];
	
	this->noIterationsPerLevel[0] = 2; //TODO -> make parameter
	for (int levelId = 1; levelId < noHierarchyLevels; levelId++)
	{
		noIterationsPerLevel[levelId] = noIterationsPerLevel[levelId - 1] + 2;
	}
	this->noIterationsPerLevel[0] = 10; //TODO -> make parameter

	float distThreshStep = distThresh / noHierarchyLevels;
	this->distThresh[noHierarchyLevels - 1] = distThresh;
	for (int levelId = noHierarchyLevels - 2; levelId >= 0; levelId--)
		this->distThresh[levelId] = this->distThresh[levelId + 1] - distThreshStep;

	this->lowLevelEngine = lowLevelEngine;

	this->noICPLevel = noICPRunTillLevel;

	this->terminationThreshold = terminationThreshold;

	this->memory_type = memoryType;

	type = tracker_type;

	// PCL viewer
	viz_icp = visualize_icp;
	if (viz_icp) {
    pc_viewer.setBackgroundColor(0, 0, 0);
    pc_viewer.addPointCloud<pcl::PointXYZRGB>(scene_cloud_pointer, "scene cloud");
    pc_viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene cloud");
    pc_viewer.addPointCloud<pcl::PointXYZRGB>(current_view_cloud_pointer, "current scan");
    pc_viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "current scan");
	}
}

ITMDepthTracker::~ITMDepthTracker(void) 
{ 
	delete this->viewHierarchy;
	delete this->sceneHierarchy;

	delete[] this->noIterationsPerLevel;
	delete[] this->distThresh;
}

void ITMDepthTracker::SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view)
{
	this->trackingState = trackingState;
	this->view = view;

	sceneHierarchy->levels[0]->intrinsics = view->calib->intrinsics_d.projectionParamsSimple.all;
	viewHierarchy->levels[0]->intrinsics = view->calib->intrinsics_d.projectionParamsSimple.all;

	// the image hierarchy allows pointers to external data at level 0
	viewHierarchy->levels[0]->depth = view->depth;
	sceneHierarchy->levels[0]->pointsMap = trackingState->pointCloud->locations;
	sceneHierarchy->levels[0]->normalsMap = trackingState->pointCloud->colours;

	scenePose = trackingState->pose_pointCloud->GetM();
	sceneInvPose = trackingState->pose_pointCloud->GetInvM();
}

void ITMDepthTracker::PrepareForEvaluation()
{
	for (int i = 1; i < viewHierarchy->noLevels; i++)
	{
		ITMTemplatedHierarchyLevel<ITMFloatImage> *currentLevelView = viewHierarchy->levels[i], *previousLevelView = viewHierarchy->levels[i - 1];
		lowLevelEngine->FilterSubsampleWithHoles(currentLevelView->depth, previousLevelView->depth);
		currentLevelView->intrinsics = previousLevelView->intrinsics * 0.5f;

		ITMSceneHierarchyLevel *currentLevelScene = sceneHierarchy->levels[i], *previousLevelScene = sceneHierarchy->levels[i - 1];
		//lowLevelEngine->FilterSubsampleWithHoles(currentLevelScene->pointsMap, previousLevelScene->pointsMap);
		//lowLevelEngine->FilterSubsampleWithHoles(currentLevelScene->normalsMap, previousLevelScene->normalsMap);
		currentLevelScene->intrinsics = previousLevelScene->intrinsics * 0.5f;
	}
}

void ITMDepthTracker::SetEvaluationParams(int levelId)
{
	this->levelId = levelId;
	this->iterationType = viewHierarchy->levels[levelId]->iterationType;
	this->sceneHierarchyLevel = sceneHierarchy->levels[0];
	this->viewHierarchyLevel = viewHierarchy->levels[levelId];
}

void ITMDepthTracker::ComputeDelta(float *step, float *nabla, float *hessian, bool shortIteration) const
{
	for (int i = 0; i < 6; i++) step[i] = 0;

	if (shortIteration)
	{
		float smallHessian[3 * 3];
		for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) smallHessian[r + c * 3] = hessian[r + c * 6];

		ORUtils::Cholesky cholA(smallHessian, 3);
		cholA.Backsub(step, nabla);
	}
	else
	{
		ORUtils::Cholesky cholA(hessian, 6);
		cholA.Backsub(step, nabla);
	}
}

bool ITMDepthTracker::HasConverged(float *step) const
{
	float stepLength = 0.0f;
	for (int i = 0; i < 6; i++) stepLength += step[i] * step[i];

	if (sqrt(stepLength) / 6 < terminationThreshold) return true; //converged

	return false;
}

void ITMDepthTracker::ApplyDelta(const Matrix4f & para_old, const float *delta, Matrix4f & para_new) const
{
	float step[6];

	switch (iterationType)
	{
	case TRACKER_ITERATION_ROTATION:
		step[0] = (float)(delta[0]); step[1] = (float)(delta[1]); step[2] = (float)(delta[2]);
		step[3] = 0.0f; step[4] = 0.0f; step[5] = 0.0f;
		break;
	case TRACKER_ITERATION_TRANSLATION:
		step[0] = 0.0f; step[1] = 0.0f; step[2] = 0.0f;
		step[3] = (float)(delta[0]); step[4] = (float)(delta[1]); step[5] = (float)(delta[2]);
		break;
	default:
	case TRACKER_ITERATION_BOTH:
		step[0] = (float)(delta[0]); step[1] = (float)(delta[1]); step[2] = (float)(delta[2]);
		step[3] = (float)(delta[3]); step[4] = (float)(delta[4]); step[5] = (float)(delta[5]);
		break;
	}

	Matrix4f Tinc;

	Tinc.m00 = 1.0f;		Tinc.m10 = step[2];		Tinc.m20 = -step[1];	Tinc.m30 = step[3];
	Tinc.m01 = -step[2];	Tinc.m11 = 1.0f;		Tinc.m21 = step[0];		Tinc.m31 = step[4];
	Tinc.m02 = step[1];		Tinc.m12 = -step[0];	Tinc.m22 = 1.0f;		Tinc.m32 = step[5];
	Tinc.m03 = 0.0f;		Tinc.m13 = 0.0f;		Tinc.m23 = 0.0f;		Tinc.m33 = 1.0f;

	para_new = Tinc * para_old;
}

void ITMDepthTracker::TrackCamera(ITMTrackingState *trackingState, const ITMView *view)
{
	this->SetEvaluationData(trackingState, view);
	this->PrepareForEvaluation();
	Matrix4f initPose = trackingState->pose_d->GetM();

	float f_old = 1e10, f_new;  // error metric
	int noValidPoints_new;
	Vector4f* matches;

	float hessian_good[6 * 6], hessian_new[6 * 6], A[6 * 6];
	float nabla_good[6], nabla_new[6];
	float step[6];

	// Libpointmatcher
	if (type == ITMLibSettings::TRACKER_LPM) {
    if (memory_type == MEMORYDEVICE_CPU) {
      this->SetEvaluationParams(0);
      Matrix4f approxInvPose = trackingState->pose_d->GetInvM();
      approxInvPose = this->getLPMICPTF(approxInvPose);
      trackingState->pose_d->SetInvM(approxInvPose);
      trackingState->pose_d->Coerce();
      approxInvPose = trackingState->pose_d->GetInvM();
      return;
    } else {
      std::cout << "LPM tracker only supported for CPU architecture!" << std::endl;
      exit(1);
    }
	}

	for (int levelId = viewHierarchy->noLevels - 1; levelId >= noICPLevel; levelId--)
	{
		this->SetEvaluationParams(levelId);
		if (iterationType == TRACKER_ITERATION_NONE) continue;

		// T_g,k =  approxInvPose
		Matrix4f approxInvPose = trackingState->pose_d->GetInvM();
		ITMPose lastKnownGoodPose(*(trackingState->pose_d));
		f_old = 1e20f;
		float lambda = 1.0;

		// Libnabo
		Eigen::MatrixXf M;
		if (type == ITMLibSettings::TRACKER_NABO) {
		  if (memory_type == MEMORYDEVICE_CPU) {
    //      delete nns;
        Eigen::MatrixXf M1 = ITMVectorToEigenMatrix(
            sceneHierarchy->levels[0]->pointsMap->GetData(MemoryDeviceType(memory_type)),
            sceneHierarchy->levels[0]->pointsMap->noDims);
        M = M1;
		  } else {
		    std::cout << "NABO tracker only supported for CPU architecture!" << std::endl;
		    exit(1);
		  }
    } else {
      Eigen::MatrixXf M1(1, 1);
      M = M1;
    }
		nns.reset(Nabo::NNSearchF::createKDTreeLinearHeap(M, M.rows()));
    //    nns = Nabo::NNSearchF::createKDTreeLinearHeap(M, M.rows());
    std::cout << "Tree Prepared......................................... " << nns->cloud.cols() << std::endl;

    // Libpointmatcher
    if (type == ITMLibSettings::TRACKER_LPM_HIERARCHY) {
      if (memory_type == MEMORYDEVICE_CPU) {
        std::cout << "Level ID: " << levelId << std::endl;
        approxInvPose = this->getLPMICPTF(approxInvPose);
        trackingState->pose_d->SetInvM(approxInvPose);
        trackingState->pose_d->Coerce();
        approxInvPose = trackingState->pose_d->GetInvM();
      } else {
        std::cout << "LPM_HIERARCHY tracker only supported for CPU architecture!" << std::endl;
        exit(1);
      }
    } else {
      for (int iterNo = 0; iterNo < noIterationsPerLevel[levelId]; iterNo++)
      {
        std::cout << "[ Level ID, Iteration no ]: " << "[ " << levelId << ", "
            << iterNo << " ]" << std::endl;
        // evaluate error function and gradients
        std::pair<Vector4f*, int> res = this->ComputeGandH(f_new, nabla_new, hessian_new, approxInvPose);
        matches = res.first;
        noValidPoints_new = res.second;

        if (viz_icp) {
          std::vector<Matrix4f*> tf_chain{&approxInvPose, &scenePose};
          // visualize matches
          visualizeTracker(this->sceneHierarchyLevel->pointsMap, this->viewHierarchyLevel->depth,
                           this->viewHierarchyLevel->intrinsics, matches, memory_type, tf_chain);
        }

        std::cout << "here0" << std::endl;
        // check if error increased. If so, revert
        if ((noValidPoints_new <= 0)||(f_new > f_old)) {
          trackingState->pose_d->SetFrom(&lastKnownGoodPose);
          approxInvPose = trackingState->pose_d->GetInvM();
          lambda *= 10.0f;
        } else {
          lastKnownGoodPose.SetFrom(trackingState->pose_d);
          f_old = f_new;

          for (int i = 0; i < 6*6; ++i) hessian_good[i] = hessian_new[i] / noValidPoints_new;
          for (int i = 0; i < 6; ++i) nabla_good[i] = nabla_new[i] / noValidPoints_new;
          lambda /= 10.0f;
        }
        for (int i = 0; i < 6*6; ++i) A[i] = hessian_good[i];
        for (int i = 0; i < 6; ++i) A[i+i*6] *= 1.0f + lambda;

        // compute a new step and make sure we've got an SE3
        ComputeDelta(step, nabla_good, A, iterationType != TRACKER_ITERATION_BOTH);
        ApplyDelta(approxInvPose, step, approxInvPose);
        trackingState->pose_d->SetInvM(approxInvPose);
        trackingState->pose_d->Coerce();
        approxInvPose = trackingState->pose_d->GetInvM();

        // if step is small, assume it's going to decrease the error and finish
        bool converged = HasConverged(step);

        // Visualization
        if (viz_icp) {
          std::cout << "here1" << std::endl;
          std::vector<Matrix4f*> tf_chain{&approxInvPose, &scenePose};
          // visualize TF update
          std::cout << "vizing updated tf" << std::endl;
          visualizeTracker(
              this->sceneHierarchyLevel->pointsMap, this->viewHierarchyLevel->depth,
              this->viewHierarchyLevel->intrinsics, memory_type, tf_chain, converged);
        }

        // if step is small, assume it's going to decrease the error and finish
        if (converged) break;
      } // iterations
    } // memory_type
	} // level change

	// outlier detection using Mahalanobis distance
	Eigen::Vector3f init_att = initPose.getRot().toEigen().eulerAngles(0, 1, 2);
	Eigen::Vector3f init_pos(initPose.getTrans().x, initPose.getTrans().y, initPose.getTrans().z);
	Eigen::VectorXd init_state(6);
	init_state << init_pos.cast<double>(), init_att.cast<double>();
	Eigen::Vector3f new_att = trackingState->pose_d->GetR().toEigen().eulerAngles(0, 1, 2);
  Eigen::Vector3f new_pos(
      trackingState->pose_d->GetT().x, trackingState->pose_d->GetT().y, trackingState->pose_d->GetT().z);
  Eigen::VectorXd new_state(6);
  new_state << new_pos.cast<double>(), new_att.cast<double>();
	Eigen::Matrix<double,6,6> odom_cov = ((ITMViewOdometry*)view)->odom->cov.toEigen();
	double dist = (new_state-init_state).transpose()*odom_cov.inverse()*(new_state-init_state);
	std::cout << "Outlier distance: " << dist << std::endl;
	if (dist > 0.003) { // outlier
	  trackingState->pose_d->SetM(initPose);
	  gp_outlier_dist.push_back(0.0);
	} else {
	  gp_outlier_dist.push_back(dist);
	}
	// Un-comment to plot outlier distances
//  gp << "plot '-' with lines title 'outlier distance'\n";
//  gp.send1d(gp_outlier_dist);
}


const Eigen::MatrixXf ITMDepthTracker::ITMVectorToEigenMatrix(
    const Vector4f* vector, const Vector2i dim) {
  Eigen::MatrixXf m(3, dim.height * dim.width);
  Vector4f point;
  for (int i = 0; i < dim.width * dim.height; ++i) {
#ifndef COMPILE_WITHOUT_CUDA
    ITMSafeCall(cudaMemcpy(&point, &vector[i], sizeof(Vector4f), cudaMemcpyDeviceToHost));
#else
    point = vector[i];
#endif
    m(0, i) = point.x;
    m(1, i) = point.y;
    m(2, i) = point.z;
  }

  return m;
}

// Float4Image to PCL point cloud
void ITMDepthTracker::Float4ImagetoPclPointCloud(
    const ITMFloat4Image* im, pcl::PointCloud<pcl::PointXYZRGB>& cloud, Vector3i color,
    int memory_type) {

  cloud.clear();
  const Vector4f* v = im->GetData(MemoryDeviceType(memory_type));
  Vector4f point;
  pcl::PointXYZRGB pc_point;
  for (int i = 0; i < im->noDims.width * im->noDims.height; ++i) {

#ifndef COMPILE_WITHOUT_CUDA
    ITMSafeCall(cudaMemcpy(&point, &v[i], sizeof(Vector4f), cudaMemcpyDeviceToHost));
#else
    point = v[i];
#endif

    pc_point.x = point.x;
    pc_point.y = point.y;
    pc_point.z = point.z;

    pc_point.r = color[0];
    pc_point.g = color[1];
    pc_point.b = color[2];

    cloud.push_back(pc_point);
  }
}


// FloatImage to PCL point cloud
void ITMDepthTracker::FloatImagetoPclPointCloud(
    const ITMFloatImage* im, pcl::PointCloud<pcl::PointXYZRGB>& cloud,
    const Vector4f intrinsics, Vector3i color, int memory_type, std::vector<Matrix4f*>& tf_chain) {

  cloud.clear();
  const float* v = im->GetData(MemoryDeviceType(memory_type));
  float point;
  pcl::PointXYZRGB pc_point;
  for (int row = 0; row < im->noDims.height; ++row) {
    for (int col = 0; col < im->noDims.width; ++col) {

#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(&point, &v[row*im->noDims.width + col], sizeof(float),
                             cudaMemcpyDeviceToHost));
#else
      point = v[row*im->noDims.width + col];
#endif

      Vector4f vec_point(point * ((float(col) - intrinsics.z) / intrinsics.x),
                         point * ((float(row) - intrinsics.w) / intrinsics.y),
                         point, 1.0);

      // apply transform chain
      for (std::vector<Matrix4f*>::iterator it = tf_chain.begin(); it != tf_chain.end(); ++it) {
        vec_point = (**it) * vec_point;
        vec_point.w = 1.0;
      }

      pc_point.x = vec_point.x;
      pc_point.y = vec_point.y;
      pc_point.z = vec_point.z;

      pc_point.r = color[0];
      pc_point.g = color[1];
      pc_point.b = color[2];

      cloud.push_back(pc_point);
    }
  }
}


// Draw ICP point matches
void ITMDepthTracker::DrawPointMatches(
    pcl::PointCloud<pcl::PointXYZRGB>& cloud, Vector4f* matches, Vector3i color) {

  std::cout << "scan cloud size: " << cloud.size() << " " << viewHierarchyLevel->depth->noDims.height * viewHierarchyLevel->depth->noDims.width << std::endl;
  for (int i = 0; i < cloud.size(); i+=10) {
    if (matches[i].w != 0.0) {
      pcl::PointXYZ m;
      m.x = matches[i].x;
      m.y = matches[i].y;
      m.z = matches[i].z;
      std::string l_string("line");
      l_string += std::to_string(i);
      if (!pcl::isFinite(cloud[i]) || !pcl::isFinite(m)) std::cout << "NOT FINITE POINT!!!!!!!!!!!" << std::endl;
      if (cloud[i].x == m.x && cloud[i].y == m.y && cloud[i].z == m.z) std::cout << "ZERO LENGTH LINE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
      if (cloud[i].x != m.x || cloud[i].y != m.y || cloud[i].z != m.z) {
        pc_viewer.addLine(cloud[i], m, color.r, color.g, color.b, l_string);
      }
    }
  }
}


// Tracker TF Update Visualization
void ITMDepthTracker::visualizeTracker(
    const ITMFloat4Image* scene, const ITMFloatImage* current_view, const Vector4f intrinsics,
    int memory_type, std::vector<Matrix4f*>& tf_chain, bool converged) {

  // scene
  Float4ImagetoPclPointCloud(scene, scene_cloud, Vector3i(255, 0, 0), memory_type);
  pc_viewer.updatePointCloud(scene_cloud_pointer, "scene cloud");

  // current view
  FloatImagetoPclPointCloud(current_view, current_view_cloud, intrinsics,
                            Vector3i(0, 0, 255), memory_type, tf_chain);
  pc_viewer.updatePointCloud(current_view_cloud_pointer, "current scan");

  // Message
  if (converged) {
    std::string msg("msg0");
    pc_viewer.addText("ICP converged", 50, 50, 30, 1.0, 1.0, 1.0, msg);
  }

  pcl_render_stop = false;
  boost::thread t(boost::bind(&ITMDepthTracker::pcl_render_loop, this));
  if (std::cin.get() == '\n') {
    std::cout << "Pressed ENTER" << std::endl;
    pcl_render_stop = true;
    std::cout << "waiting to join...." << std::endl;
    t.join();
    std::cout << "joined!!!" << std::endl;
  }
}


// Tracker Matches Visualization
void ITMDepthTracker::visualizeTracker(
    const ITMFloat4Image* scene, const ITMFloatImage* current_view,
    const Vector4f intrinsics, Vector4f* matches, int memory_type, std::vector<Matrix4f*>& tf_chain) {

//  pc_viewer.removeAllPointClouds();
  // scene
  Float4ImagetoPclPointCloud(scene, scene_cloud, Vector3i(255, 0, 0), memory_type);
  pc_viewer.updatePointCloud(scene_cloud_pointer, "scene cloud");

  // current view
  FloatImagetoPclPointCloud(current_view, current_view_cloud, intrinsics,
                            Vector3i(0, 0, 255), memory_type, tf_chain);
  pc_viewer.updatePointCloud(current_view_cloud_pointer, "current scan");

  // matches
  DrawPointMatches(current_view_cloud, matches, Vector3i(255, 255, 255));

  pcl_render_stop = false;
  boost::thread t(boost::bind(&ITMDepthTracker::pcl_render_loop, this));
  if (std::cin.get() == '\n') {
    std::cout << "Pressed ENTER" << std::endl;
    pcl_render_stop = true;
    std::cout << "waiting to join...." << std::endl;
    t.join();
    std::cout << "joined!!!" << std::endl;
    std::cout << "here -1" << std::endl;
  }
}

void ITMDepthTracker::pcl_render_loop() {
  std::cout << "stop flag: " << pcl_render_stop << std::endl;
  while (!pcl_render_stop) {
    std::cout << "SPINNING......." << std::endl;
    pc_viewer.spinOnce (100);
  }
  pc_viewer.removeAllShapes();
}
