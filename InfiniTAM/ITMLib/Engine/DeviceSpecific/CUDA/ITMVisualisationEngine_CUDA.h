// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMVisualisationEngine.h"

struct RenderingBlock;

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMVisualisationEngine_CUDA : public ITMVisualisationEngine < TVoxel, TIndex >
		{
		private:
			uint *noTotalPoints_device, *noTotalActivePoints_device, *noTotalInactivePoints_device;

		public:
			explicit ITMVisualisationEngine_CUDA(ITMScene<TVoxel, TIndex> *scene);
			~ITMVisualisationEngine_CUDA(void);

			void FindVisibleBlocks(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void CreateExpectedDepths(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void RenderImage(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMTrackingState *trackingState, ITMRenderState *renderState,
				ITMUChar4Image *outputImage, const float delta_time, IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE) const;
			void FindSurface(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
			void CreatePointCloud(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
			void CreateICPMaps(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
			void ForwardRender(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;

			ITMRenderState* CreateRenderState(const Vector2i & imgSize, const float rewind_time) const;
		};

		template<class TVoxel>
		class ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash> : public ITMVisualisationEngine < TVoxel, ITMVoxelBlockHash >
		{
		private:
			uint *noTotalPoints_device, *noTotalActivePoints_device, *noTotalInactivePoints_device;
			RenderingBlock *renderingBlockList_device;
			uint *noTotalBlocks_device;
			int *noVisibleEntries_device;
		public:
			explicit ITMVisualisationEngine_CUDA(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);
			~ITMVisualisationEngine_CUDA(void);

			void FindVisibleBlocks(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void CreateExpectedDepths(const ITMPose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
			void RenderImage(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMTrackingState *trackingState, ITMRenderState *renderState,
				ITMUChar4Image *outputImage,const float delta_time,  IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE) const;
			void FindSurface(const ITMPose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
			void CreatePointCloud(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
			void CreateICPMaps(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
			void ForwardRender(const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;

			ITMRenderState_VH* CreateRenderState(const Vector2i & imgSize, const float rewind_time) const;
		};
	}
}
