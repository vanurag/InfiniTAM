// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDenseMapper.h"

#include "../Objects/ITMRenderState_VH.h"

#include "../ITMLib.h"

using namespace ITMLib::Engine;

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel, TIndex>::ITMDenseMapper(const ITMLibSettings *settings)
{
	swappingEngine = NULL;

	switch (settings->deviceType)
	{
	case ITMLibSettings::DEVICE_CPU:
		sceneRecoEngine = new ITMSceneReconstructionEngine_CPU<TVoxel,TIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<TVoxel,TIndex>();
		break;
	case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
		sceneRecoEngine = new ITMSceneReconstructionEngine_CUDA<TVoxel,TIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CUDA<TVoxel,TIndex>();
#endif
		break;
	case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
		sceneRecoEngine = new ITMSceneReconstructionEngine_Metal<TVoxel, TIndex>();
		if (settings->useSwapping) swappingEngine = new ITMSwappingEngine_CPU<TVoxel, TIndex>();
#endif
		break;
	}
}

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel,TIndex>::~ITMDenseMapper()
{
	delete sceneRecoEngine;
	if (swappingEngine!=NULL) delete swappingEngine;
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::ResetScene(ITMScene<TVoxel,TIndex> *scene, const float rewind_time)
{
	sceneRecoEngine->ResetScene(scene, rewind_time);
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::ProcessFrame(const ITMView *view, ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState, const float delta_time)
{
	// allocation
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState, delta_time);

	// integration
	sceneRecoEngine->IntegrateIntoScene(scene, view, trackingState, renderState);

	if (swappingEngine != NULL) {
		// swapping: CPU -> GPU
	  float bla_t = sdkGetTimerValue(&renderState->timer);
		swappingEngine->IntegrateGlobalIntoLocal(scene, view, trackingState, renderState);
	  float cpu_to_gpu_time = sdkGetTimerValue(&renderState->timer) - bla_t;
	  std::cout << "Took " << cpu_to_gpu_time << " ms to transfer from CPU to GPU" << std::endl;
//	  Vector4f * blabla = trackingState->pointCloud->inactive_locations->GetData(MEMORYDEVICE_CUDA);
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
		// swapping: GPU -> CPU
		swappingEngine->SaveToGlobalMemory(scene, renderState);
		float gpu_to_cpu_time = sdkGetTimerValue(&renderState->timer) - bla_t - cpu_to_gpu_time;
    std::cout << "Took " << gpu_to_cpu_time << " ms to transfer from GPU to CPU" << std::endl;
    if (gpu_to_cpu_time > 5000) {
      std::cout << "transfer took too long!!!!!!!!!!1" << std::endl;
      exit(1);
    }
	}
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel,TIndex>::UpdateVisibleList(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel,TIndex> *scene, ITMRenderState *renderState)
{
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState, true);
}

template class ITMLib::Engine::ITMDenseMapper<ITMVoxel, ITMVoxelIndex>;
