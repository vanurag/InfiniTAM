// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMSwappingEngine.h"

namespace ITMLib
{
	namespace Engine
	{
		template<class TVoxel, class TIndex>
		class ITMSwappingEngine_CPU : public ITMSwappingEngine < TVoxel, TIndex >
		{
		public:
		  void resetInactiveLocations(Vector4f* inactiveLocations, const Vector2i imgSize, const Matrix4f M_d, const Vector4f projParams_d);
			void IntegrateGlobalIntoLocal(ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
			void SaveToGlobalMemory(ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
		};

		template<class TVoxel>
		class ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMSwappingEngine < TVoxel, ITMVoxelBlockHash >
		{
		private:
			int LoadFromGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

		public:
			// This class is currently just for debugging purposes -- swaps CPU memory to CPU memory.
			// Potentially this could stream into the host memory from somwhere else (disk, database, etc.).

			void IntegrateGlobalIntoLocal(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState);
			void SaveToGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);

			ITMSwappingEngine_CPU(void);
			~ITMSwappingEngine_CPU(void);
		};
	}
}
