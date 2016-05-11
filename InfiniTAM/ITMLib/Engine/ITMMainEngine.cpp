// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMMainEngine.h"

using namespace ITMLib::Engine;

cv::viz::Viz3d ITMMainEngine::viz_window_ = cv::viz::Viz3d("ITM Tracking Pose");
cv::Affine3f ITMMainEngine::viz_itm_pose_ = cv::Affine3f();

ITMMainEngine::ITMMainEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d)
  : viz_key_event(cv::viz::KeyboardEvent::Action::KEY_DOWN, "A", cv::viz::KeyboardEvent::ALT, 1), pc_viewer("Cloud visualizer"), pcl_cloud_pointer(&pcl_cloud)
{
	// create all the things required for marching cubes and mesh extraction
	// - uses additional memory (lots!)
	static const bool createMeshingEngine = true;

	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	this->settings = settings;

	this->scene = new ITMScene<ITMVoxel, ITMVoxelIndex>(&(settings->sceneParams), settings->useSwapping, 
		settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);

	meshingEngine = NULL;
	switch (settings->deviceType)
	{
	case ITMLibSettings::DEVICE_CPU:
		lowLevelEngine = new ITMLowLevelEngine_CPU();
		viewBuilder = new ITMViewBuilder_CPU(calib);
		visualisationEngine = new ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>(scene);
		if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>();
		break;
	case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		lowLevelEngine = new ITMLowLevelEngine_CUDA();
		viewBuilder = new ITMViewBuilder_CUDA(calib);
		visualisationEngine = new ITMVisualisationEngine_CUDA<ITMVoxel, ITMVoxelIndex>(scene);
		if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CUDA<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
		lowLevelEngine = new ITMLowLevelEngine_Metal();
		viewBuilder = new ITMViewBuilder_Metal(calib);
		visualisationEngine = new ITMVisualisationEngine_Metal<ITMVoxel, ITMVoxelIndex>(scene);
		if (createMeshingEngine) meshingEngine = new ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>();
#endif
		break;
	}

	mesh = NULL;
	if (createMeshingEngine) mesh = new ITMMesh(settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU);

	Vector2i trackedImageSize = ITMTrackingController::GetTrackedImageSize(settings, imgSize_rgb, imgSize_d);

	sdkCreateTimer(&main_timer);
  sdkStartTimer(&main_timer);

	renderState_live = visualisationEngine->CreateRenderState(trackedImageSize, sdkGetTimerValue(&main_timer));
	renderState_freeview = NULL; //will be created by the visualisation engine

	denseMapper = new ITMDenseMapper<ITMVoxel, ITMVoxelIndex>(settings);
	float check_time = sdkGetTimerValue(&main_timer);
	denseMapper->ResetScene(scene, sdkGetTimerValue(&main_timer));

	imuCalibrator = new ITMIMUCalibrator_DRZ(calib->trafo_rgb_to_imu);
	tracker = ITMTrackerFactory<ITMVoxel, ITMVoxelIndex>::Instance().Make(trackedImageSize, settings, lowLevelEngine, imuCalibrator, scene);
	trackingController = new ITMTrackingController(tracker, visualisationEngine, lowLevelEngine, settings);

	trackingState = trackingController->BuildTrackingState(trackedImageSize);
	tracker->UpdateInitialPose(trackingState);

	view = NULL; // will be allocated by the view builder

	fusionActive = true;
	mainProcessingActive = true;

	// VIZ
	viz_window_.registerKeyboardCallback(VizKeyboardCallback);
	viz_window_.setWindowSize(cv::Size(600, 600));
  viz_window_.showWidget("ITM Tracking Pose", cv::viz::WCoordinateSystem(100.0));

  // PCL
  pc_viewer.setBackgroundColor(0, 0, 0);
  pc_viewer.addPointCloud<pcl::PointXYZRGB>(pcl_cloud_pointer, "cloud");
  pc_viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

  // ROS
  pubITMPose = nh.advertise<geometry_msgs::TransformStamped>("itm/pose", 1);
}

ITMMainEngine::~ITMMainEngine()
{
	delete renderState_live;
	if (renderState_freeview!=NULL) delete renderState_freeview;

	delete scene;

	delete denseMapper;
	sdkDeleteTimer(&main_timer);
	delete trackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	delete trackingState;
	if (view != NULL) delete view;

	delete visualisationEngine;

	if (meshingEngine != NULL) delete meshingEngine;

	if (mesh != NULL) delete mesh;
}

ITMMesh* ITMMainEngine::UpdateMesh(void)
{
	if (mesh != NULL) meshingEngine->MeshScene(mesh, scene);
	return mesh;
}

void ITMMainEngine::SaveSceneToMesh(const char *objFileName)
{
	if (mesh == NULL) return;
	meshingEngine->MeshScene(mesh, scene);
	mesh->WriteSTL(objFileName);
}

void ITMMainEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
  // prepare image and turn it into a depth image
  viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter,settings->modelSensorNoise);

  if (!mainProcessingActive) return;

  // tracking
  trackingController->Track(trackingState, view);

  // publish ITM tracker pose
  PublishROSPoseMsg();

  // VIZ ITM Tracker estimate
  VisualizeCameraPose();

  // fusion
  if (fusionActive) denseMapper->ProcessFrame(view, trackingState, scene, renderState_live, settings->deltaTime);

  // raycast to renderState_live for tracking and free visualisation
  trackingController->Prepare(trackingState, view, renderState_live);
}

void ITMMainEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement)
{
	// prepare image and turn it into a depth image
	if (imuMeasurement==NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter,settings->modelSensorNoise);
	else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

	if (!mainProcessingActive) return;

	// tracking
	trackingController->Track(trackingState, view);

	// publish ITM tracker pose
  PublishROSPoseMsg();

  // VIZ ITM Tracker estimate
	VisualizeCameraPose();

	// fusion
	if (fusionActive) denseMapper->ProcessFrame(view, trackingState, scene, renderState_live, settings->deltaTime);

	// raycast to renderState_live for tracking and free visualisation
	trackingController->Prepare(trackingState, view, renderState_live);
}

void ITMMainEngine::ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMOdometryMeasurement *odomMeasurement)
{
  // prepare image and turn it into a depth image
  if (odomMeasurement==NULL) viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter,settings->modelSensorNoise);
  else viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, odomMeasurement);

  if (!mainProcessingActive) return;

  // tracking
  trackingController->Track(trackingState, view);

  // publish ITM tracker pose
  PublishROSPoseMsg();

  // VIZ ITM Tracker estimate
  VisualizeCameraPose();

  // fusion
  if (fusionActive) denseMapper->ProcessFrame(view, trackingState, scene, renderState_live, settings->deltaTime);

//  Vector4f * blabla = trackingState->pointCloud->inactive_locations->GetData(MEMORYDEVICE_CUDA);
//  //  visualizePcl(blabla, trackingState->pointCloud->inactive_locations->noDims.x * trackingState->pointCloud->inactive_locations->noDims.y);
//    std::cout << "balsdasldsa: " << std::endl;
//    Vector4f pix;
//    for (int i = 0; i < 120000; ++i) {
//  #ifndef COMPILE_WITHOUT_CUDA
//      ITMSafeCall(cudaMemcpy(&pix, &blabla[i], sizeof(Vector4f), cudaMemcpyDeviceToHost));
//  #else
//      pix = blabla[i];
//  #endif
//      if (pix.w > 0) {
//        std::cout << "pix: " << (float)pix.x << ", " << (float)pix.y << ", " << (float)pix.z << std::endl;
//      }
//    }

  // raycast to renderState_live for tracking and free visualisation
  trackingController->Prepare(trackingState, view, renderState_live);
}

// VIZ ITM Tracker camera pose estimate
void ITMMainEngine::VisualizeCameraPose() {
  Matrix4f itm_pose = trackingState->pose_d->GetInvM()*view->calib->trafo_rgb_to_depth.calib*view->calib->trafo_rgb_to_imu.calib_inv;
//  cv::Mat pose_mat(3, 3, CV_32F);
  cv::Matx<float, 3, 3> pose_mat;
  float* mat_pointer = (float*)pose_mat.val;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      mat_pointer[3*row + col] = itm_pose(col, row);
    }
  }
  viz_itm_pose_.rotation(pose_mat);
  viz_itm_pose_.translation(cv::Vec3f(100.0*itm_pose.m30, 100.0*itm_pose.m31, 100.0*itm_pose.m32));
  viz_window_.setWidgetPose("ITM Tracking Pose", viz_itm_pose_);
  viz_window_.spinOnce(1, true);
}

// Publish ITM ROS pose message
void ITMMainEngine::PublishROSPoseMsg() {
  if(pubITMPose.getNumSubscribers() > 0){
    ITMPoseMsg.header.stamp = ros::Time::now();

    Matrix4f pose_imu = view->calib->trafo_rgb_to_imu.calib * view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->GetM();
    Vector3f t_inv = pose_imu.getRot().t() * (-1.0 * pose_imu.getTrans());
    ITMPoseMsg.transform.translation.x = t_inv.x;
    ITMPoseMsg.transform.translation.y = t_inv.y;
    ITMPoseMsg.transform.translation.z = t_inv.z;
    Matrix3f r = pose_imu.getRot();
    MPD R(pose_imu.m00, pose_imu.m10, pose_imu.m20,
          pose_imu.m01, pose_imu.m11, pose_imu.m21,
          pose_imu.m02, pose_imu.m12, pose_imu.m22);
    QPD q(R);
    ITMPoseMsg.transform.rotation.x = q.x(); // JPL form
    ITMPoseMsg.transform.rotation.y = q.y();
    ITMPoseMsg.transform.rotation.z = q.z();
    ITMPoseMsg.transform.rotation.w = q.w();

    pubITMPose.publish(ITMPoseMsg);
  }
}


Vector2i ITMMainEngine::GetImageSize(void) const
{
	return renderState_live->raycastImage->noDims;
}

void ITMMainEngine::GetImage(ITMUChar4Image *out, GetImageType getImageType, ITMPose *pose, ITMIntrinsics *intrinsics)
{
	if (view == NULL) return;

	out->Clear();

	switch (getImageType)
	{
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
		out->ChangeDims(view->rgb->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) 
			out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
		out->ChangeDims(view->depth->noDims);
		if (settings->trackerType==ITMLib::Objects::ITMLibSettings::TRACKER_WICP)
		{
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depthUncertainty->UpdateHostFromDevice();
			ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>::WeightToUchar4(out, view->depthUncertainty);
		}
		else
		{
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
			ITMVisualisationEngine<ITMVoxel, ITMVoxelIndex>::DepthToUchar4(
			    out, view->depth, Vector2f(settings->sceneParams.viewFrustum_min,
			                               settings->sceneParams.viewFrustum_max));
		}

		break;
	case ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH_WITH_RGB:
    out->ChangeDims(view->depth->noDims);
    if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
      out->SetFrom(view->rgb_d, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
    else out->SetFrom(view->rgb_d, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
    break;
	case ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST:
	{
		ORUtils::Image<Vector4u> *srcImage = renderState_live->raycastImage;
		out->ChangeDims(srcImage->noDims);
		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);	
		break;
	}
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
	case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
	{
		IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
		if (getImageType == ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
		else if (getImageType == ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL) type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
		if (renderState_freeview == NULL) renderState_freeview = visualisationEngine->CreateRenderState(out->noDims, sdkGetTimerValue(&main_timer));

		visualisationEngine->FindVisibleBlocks(pose, intrinsics, renderState_freeview);
		visualisationEngine->CreateExpectedDepths(pose, intrinsics, renderState_freeview);
		visualisationEngine->RenderImage(pose, intrinsics, trackingState, renderState_freeview, renderState_freeview->raycastImage, settings->deltaTime, type);
//    Vector4f * blabla = renderState_freeview->inactiveRaycastResult->GetData(MEMORYDEVICE_CUDA);
//    std::cout << "balsdasldsa: " << std::endl;
//    Vector4f pix;
//    for (int i = 0; i < 120000; ++i) {
//  #ifndef COMPILE_WITHOUT_CUDA
//      ITMSafeCall(cudaMemcpy(&pix, &blabla[i], sizeof(Vector4f), cudaMemcpyDeviceToHost));
//  #else
//      pix = blabla[i];
//  #endif
//      if (pix.w > 0) {
//        std::cout << "pix: " << pix << std::endl;
//      }
//    }

		if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
			out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
		else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		break;
	}
	case ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN:
		break;
	};
}

void ITMMainEngine::turnOnIntegration() { fusionActive = true; }
void ITMMainEngine::turnOffIntegration() { fusionActive = false; }
void ITMMainEngine::turnOnMainProcessing() { mainProcessingActive = true; }
void ITMMainEngine::turnOffMainProcessing() { mainProcessingActive = false; }


// PCL Visualization
void ITMMainEngine::visualizePcl(const Vector4f* pcl, const int cloudSize) {

  pcl_cloud.clear();

  pcl::PointXYZRGB pc_point;
  Vector4f point;
  for (int i = 0; i < cloudSize; ++i){
#ifndef COMPILE_WITHOUT_CUDA
    ITMSafeCall(cudaMemcpy(&point, &pcl[i], sizeof(Vector4f), cudaMemcpyDeviceToHost));
#else
    point = pcl[i];
#endif
    if (point.w > 0) {
      pc_point.x = point.x;
      pc_point.y = point.y;
      pc_point.z = point.z;

      pc_point.r = 255;
      pc_point.g = 0;
      pc_point.b = 0;

      pcl_cloud.push_back(pc_point);
    }
  }

  pc_viewer.updatePointCloud(pcl_cloud_pointer, "cloud");

  pcl_render_stop = false;
  boost::thread t(boost::bind(&ITMMainEngine::pcl_render_loop, this));
  if (std::cin.get() == '\n') {
    std::cout << "Pressed ENTER" << std::endl;
    pcl_render_stop = true;
    std::cout << "waiting to join...." << std::endl;
    t.join();
  }
}

void ITMMainEngine::pcl_render_loop() {
  std::cout << "stop flag: " << pcl_render_stop << std::endl;
  while (!pcl_render_stop) {
    std::cout << "SPINNING......." << std::endl;
    pc_viewer.spinOnce (100);
  }
  pc_viewer.removeAllShapes();
}

