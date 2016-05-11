// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSwappingEngine_CPU.h"
#include "../../DeviceAgnostic/ITMSwappingEngine.h"
#include "../../../Objects/ITMRenderState_VH.h"

using namespace ITMLib::Engine;

template<class TVoxel>
ITMSwappingEngine_CPU<TVoxel,ITMVoxelBlockHash>::ITMSwappingEngine_CPU(void)
{
}

template<class TVoxel>
ITMSwappingEngine_CPU<TVoxel,ITMVoxelBlockHash>::~ITMSwappingEngine_CPU(void)
{
}

template<class TVoxel>
int ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash>::LoadFromGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
{
	ITMGlobalCache<TVoxel> *globalCache = scene->globalCache;

	ITMHashSwapState *swapStates = globalCache->GetSwapStates(false);

	int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(false);

	TVoxel *syncedVoxelBlocks_global = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_global = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_global = globalCache->GetNeededEntryIDs(false);

	int noTotalEntries = globalCache->noTotalEntries;

	int noNeededEntries = 0;
	for (int entryId = 0; entryId < noTotalEntries; entryId++)
	{
		if (noNeededEntries >= SDF_TRANSFER_BLOCK_NUM) break;
		if (swapStates[entryId].state == 1)
		{
			neededEntryIDs_local[noNeededEntries] = entryId;
			noNeededEntries++;
		}
	}

	// would copy neededEntryIDs_local into neededEntryIDs_global here

	if (noNeededEntries > 0)
	{
		memset(syncedVoxelBlocks_global, 0, noNeededEntries * SDF_BLOCK_SIZE3 * sizeof(TVoxel));
		memset(hasSyncedData_global, 0, noNeededEntries * sizeof(bool));
		for (int i = 0; i < noNeededEntries; i++)
		{
			int entryId = neededEntryIDs_global[i];

			if (globalCache->HasStoredData(entryId))
			{
				hasSyncedData_global[i] = true;
				memcpy(syncedVoxelBlocks_global + i * SDF_BLOCK_SIZE3, globalCache->GetStoredVoxelBlock(entryId), SDF_BLOCK_SIZE3 * sizeof(TVoxel));
			}
		}
	}

	// would copy syncedVoxelBlocks_global and hasSyncedData_global and syncedVoxelBlocks_local and hasSyncedData_local here

	return noNeededEntries;
}

void resetInactiveLocations(Vector4f* inactiveLocations, const Vector2i imgSize, const Matrix4f M_d, const Vector4f projParams_d) {
  for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId) {
    int pixelLoc = forwardProjectPoint(inactiveLocations[locId], M_d, projParams_d, imgSize);
    if (pixelLoc < 0) inactiveLocations[locId] = Vector4f(0,0,0,0);
  }
}

template<class TVoxel>
void ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash>::IntegrateGlobalIntoLocal(ITMScene<TVoxel, ITMVoxelBlockHash> *scene,
    const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
{
	ITMGlobalCache<TVoxel> *globalCache = scene->globalCache;

	ITMHashEntry *hashTable = scene->index.GetEntries();

	ITMHashSwapState *swapStates = globalCache->GetSwapStates(false);

	TVoxel *syncedVoxelBlocks_local = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_local = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(false);

	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();

	Vector4f * inactiveLocations = trackingState->pointCloud->inactive_locations->GetData(MEMORYDEVICE_CPU);
	Matrix4f M_d = trackingState->pose_d->GetM();
  Vector4f projParams_d = view->calib->intrinsics_d.projectionParamsSimple.all;
  Vector2i imgSize = view->depth->noDims;
  float voxelSize = scene->sceneParams->voxelSize;

  resetInactiveLocations(inactiveLocations, trackingState->pointCloud->inactive_locations->noDims, M_d, projParams_d);

	int noNeededEntries = this->LoadFromGlobalMemory(scene);

	int maxW = scene->sceneParams->maxW;

	for (int i = 0; i < noNeededEntries; i++)
	{
		int entryDestId = neededEntryIDs_local[i];

		if (hasSyncedData_local[i])
		{
			TVoxel *inactiveVB = syncedVoxelBlocks_local + i * SDF_BLOCK_SIZE3;
//			TVoxel *dstVB = localVBA + hashTable[entryDestId].ptr * SDF_BLOCK_SIZE3;

			Vector3i globalPos;
			globalPos.x = hashTable[entryDestId].pos.x;
      globalPos.y = hashTable[entryDestId].pos.y;
      globalPos.z = hashTable[entryDestId].pos.z;
      globalPos *= SDF_BLOCK_SIZE;

      for (int z = 0; z < SDF_BLOCK_SIZE; z++) for (int y = 0; y < SDF_BLOCK_SIZE; y++) for (int x = 0; x < SDF_BLOCK_SIZE; x++)
      {
        Vector4f pt_model; int locId;

        locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
        if (inactiveVB[locId].w_depth == 0) continue;

        pt_model.x = (float)(globalPos.x + x) * voxelSize;
        pt_model.y = (float)(globalPos.y + y) * voxelSize;
        pt_model.z = (float)(globalPos.z + z) * voxelSize;
        pt_model.w = 1.0f;

        // project into camera image and get x,y
        int pixelLoc = forwardProjectPoint(pt_model, M_d, projParams_d, imgSize);
        if (pixelLoc >= 0) {
          inactiveLocations[pixelLoc] = pt_model;
        }

//        CombineVoxelInformation<TVoxel::hasColorInformation, TVoxel>::compute(srcVB[locId], dstVB[vIdx], maxW, sdkGetTimerValue(&renderState->timer));
      }
		}

		swapStates[entryDestId].state = 2;
	}
}

template<class TVoxel>
void ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash>::SaveToGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState)
{
	ITMGlobalCache<TVoxel> *globalCache = scene->globalCache;

	ITMHashSwapState *swapStates = globalCache->GetSwapStates(false);

	ITMHashEntry *hashTable = scene->index.GetEntries();
	uchar *entriesVisibleType = ((ITMRenderState_VH*)renderState)->GetEntriesVisibleType();

	TVoxel *syncedVoxelBlocks_local = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_local = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(false);

	TVoxel *syncedVoxelBlocks_global = globalCache->GetSyncedVoxelBlocks(false);
	bool *hasSyncedData_global = globalCache->GetHasSyncedData(false);
	int *neededEntryIDs_global = globalCache->GetNeededEntryIDs(false);

	TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
	int *voxelAllocationList = scene->localVBA.GetAllocationList();

	int noTotalEntries = globalCache->noTotalEntries;
	
	int noNeededEntries = 0;
	int noAllocatedVoxelEntries = scene->localVBA.lastFreeBlockId;

	for (int entryDestId = 0; entryDestId < noTotalEntries; entryDestId++)
	{
		if (noNeededEntries >= SDF_TRANSFER_BLOCK_NUM) break;

		int localPtr = hashTable[entryDestId].ptr;
		ITMHashSwapState &swapState = swapStates[entryDestId];

		if (swapState.state == 2 && localPtr >= 0 && entriesVisibleType[entryDestId] == 0)
		{
			TVoxel *localVBALocation = localVBA + localPtr * SDF_BLOCK_SIZE3;

			neededEntryIDs_local[noNeededEntries] = entryDestId;

			hasSyncedData_local[noNeededEntries] = true;
			memcpy(syncedVoxelBlocks_local + noNeededEntries * SDF_BLOCK_SIZE3, localVBALocation, SDF_BLOCK_SIZE3 * sizeof(TVoxel));

			swapStates[entryDestId].state = 0;

			int vbaIdx = noAllocatedVoxelEntries;
			if (vbaIdx < SDF_BUCKET_NUM - 1)
			{
				noAllocatedVoxelEntries++;
				voxelAllocationList[vbaIdx + 1] = localPtr;
				hashTable[entryDestId].ptr = -1;

				for (int i = 0; i < SDF_BLOCK_SIZE3; i++) localVBALocation[i] = TVoxel();
			}

			noNeededEntries++;
		}
	}

	scene->localVBA.lastFreeBlockId = noAllocatedVoxelEntries;

	// would copy neededEntryIDs_local, hasSyncedData_local and syncedVoxelBlocks_local into *_global here

	if (noNeededEntries > 0)
	{
		for (int entryId = 0; entryId < noNeededEntries; entryId++)
		{
			if (hasSyncedData_global[entryId])
				globalCache->SetStoredData(neededEntryIDs_global[entryId], syncedVoxelBlocks_global + entryId * SDF_BLOCK_SIZE3);
		}
	}
}

template class ITMLib::Engine::ITMSwappingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
