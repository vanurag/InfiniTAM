// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMTrackingController.h"

#include "../Objects/ITMRenderState_VH.h"

#include "../ITMLib.h"

using namespace ITMLib::Engine;

void ITMTrackingController::Track(ITMTrackingState *trackingState, const ITMView *view)
{
	if (trackingState->age_pointCloud!=-1) {
	  tracker->TrackCamera(trackingState, view);
//	  loopClosureDetector->DetectLoopClosure(trackingState, view);
	}

	trackingState->requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;
}

void ITMTrackingController::Prepare(ITMTrackingState *trackingState, const ITMView *view, ITMRenderState *renderState)
{
	//render for tracking

	if (settings->trackerType == ITMLibSettings::TRACKER_COLOR)
	{
		ITMPose pose_rgb(view->calib->trafo_rgb_to_depth.calib_inv * trackingState->pose_d->GetM());
		visualisationEngine->CreateExpectedDepths(&pose_rgb, &(view->calib->intrinsics_rgb), renderState);
		visualisationEngine->CreatePointCloud(view, trackingState, renderState, settings->skipPoints);
		trackingState->age_pointCloud = 0;
	}
	else
	{
		visualisationEngine->CreateExpectedDepths(trackingState->pose_d, &(view->calib->intrinsics_d), renderState);

		if (trackingState->requiresFullRendering)
		{
		  std::cout << "full rendering..." << std::endl;
			visualisationEngine->CreateICPMaps(view, trackingState, renderState);
//      Vector4f * blabla = renderState->inactiveRaycastResult->GetData(MEMORYDEVICE_CUDA);
//      int num_read = 0;
//      Vector4f pix;
//      for (int i = 0; i < renderState->inactiveRaycastResult->noDims.x * renderState->inactiveRaycastResult->noDims.y; ++i) {
//    #ifndef COMPILE_WITHOUT_CUDA
//        ITMSafeCall(cudaMemcpy(&pix, &blabla[i], sizeof(Vector4f), cudaMemcpyDeviceToHost));
//    #else
//        pix = blabla[i];
//    #endif
//        if (pix.w > 0) {
//  //        std::cout << "pix: " << (float)pix.x << ", " << (float)pix.y << ", " << (float)pix.z << std::endl;
//          num_read++;
//        }
//      }
//      std::cout << "ICP maps contain: " << num_read << std::endl;
			std::cout << "full rendering done!!" << std::endl;
			trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
			if (trackingState->age_pointCloud==-1) trackingState->age_pointCloud=-2;
			else trackingState->age_pointCloud = 0;
		}
		else
		{
		  std::cout << "forward rendering..." << std::endl;
			visualisationEngine->ForwardRender(view, trackingState, renderState);
			trackingState->age_pointCloud++;
		}
	}
}
