// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthTracker_CUDA.h"
#include "ITMCUDAUtils.h"
#include "../../DeviceAgnostic/ITMDepthTracker.h"
#include "../../../../ORUtils/CUDADefines.h"

using namespace ITMLib::Engine;

struct ITMDepthTracker_CUDA::AccuCell {
	int numPoints;
	float f;
	float g[6];
	float h[6+5+4+3+2+1];
};

struct ITMDepthTracker_KernelParameters {
	ITMDepthTracker_CUDA::AccuCell *accu;
	Vector4f* matches;
	float *depth;
	Matrix4f approxInvPose;
	Vector4f *pointsMap;
	Vector4f *normalsMap;
	Vector4f sceneIntrinsics;
	Vector2i sceneImageSize;
	Matrix4f scenePose;
	Vector4f viewIntrinsics;
	Vector2i viewImageSize;
	float distThresh;
};

template<bool shortIteration, bool rotationOnly>
__global__ void depthTrackerOneLevel_g_rt_device(ITMDepthTracker_KernelParameters para);

// host methods

ITMDepthTracker_CUDA::ITMDepthTracker_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel,
	float distThresh, float terminationThreshold, bool visualize_icp, const ITMLowLevelEngine *lowLevelEngine)
	:ITMDepthTracker(imgSize, trackingRegime, noHierarchyLevels, noICPRunTillLevel, distThresh, terminationThreshold, visualize_icp, lowLevelEngine, MEMORYDEVICE_CUDA)
{
	ITMSafeCall(cudaMallocHost((void**)&accu_host, sizeof(AccuCell)));
	ITMSafeCall(cudaMalloc((void**)&accu_device, sizeof(AccuCell)));
}

ITMDepthTracker_CUDA::~ITMDepthTracker_CUDA(void)
{
	ITMSafeCall(cudaFreeHost(accu_host));
	ITMSafeCall(cudaFree(accu_device));
}

std::pair<Vector4f*, int> ITMDepthTracker_CUDA::ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{
	Vector4f *pointsMap = sceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CUDA);
	Vector4f *normalsMap = sceneHierarchyLevel->normalsMap->GetData(MEMORYDEVICE_CUDA);
	Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;
	Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;

	float *depth = viewHierarchyLevel->depth->GetData(MEMORYDEVICE_CUDA);
	Vector4f viewIntrinsics = viewHierarchyLevel->intrinsics;
	Vector2i viewImageSize = viewHierarchyLevel->depth->noDims;

	if (iterationType == TRACKER_ITERATION_NONE) {
		return std::make_pair(new Vector4f, 0);
	}

	bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);

	int noPara = shortIteration ? 3 : 6;

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)viewImageSize.x / (float)blockSize.x), (int)ceil((float)viewImageSize.y / (float)blockSize.y));

	ITMSafeCall(cudaMemset(accu_device, 0, sizeof(AccuCell)));
	Vector4f* device_matches = new Vector4f;
	ITMSafeCall(cudaMalloc((void**)&device_matches, viewImageSize.height * viewImageSize.width * sizeof(Vector4f)));
	ITMSafeCall(cudaMemset(device_matches, 0, viewImageSize.height * viewImageSize.width * sizeof(Vector4f)));

	struct ITMDepthTracker_KernelParameters args;
	args.accu = accu_device;
	args.matches = device_matches;
	args.depth = depth;
	args.approxInvPose = approxInvPose;
	args.pointsMap = pointsMap;
	args.normalsMap = normalsMap;
	args.sceneIntrinsics = sceneIntrinsics;
	args.sceneImageSize = sceneImageSize;
	args.scenePose = scenePose;
	args.viewIntrinsics = viewIntrinsics;
	args.viewImageSize = viewImageSize;
	args.distThresh = distThresh[levelId];

	switch (iterationType)
	{
	case TRACKER_ITERATION_ROTATION:
		depthTrackerOneLevel_g_rt_device<true, true> << <gridSize, blockSize >> >(args);
		break;
	case TRACKER_ITERATION_TRANSLATION:
		depthTrackerOneLevel_g_rt_device<true, false> << <gridSize, blockSize >> >(args);
		break;
	case TRACKER_ITERATION_BOTH:
		depthTrackerOneLevel_g_rt_device<false, false> << <gridSize, blockSize >> >(args);
		break;
	default: break;
	}

	ITMSafeCall(cudaMemcpy(accu_host, accu_device, sizeof(AccuCell), cudaMemcpyDeviceToHost));
	Vector4f* host_matches = new Vector4f;
	ITMSafeCall(cudaMallocHost((void**)&host_matches, viewImageSize.height * viewImageSize.width * sizeof(Vector4f)));
	ITMSafeCall(cudaMemcpy(host_matches, device_matches,
			viewImageSize.height * viewImageSize.width * sizeof(Vector4f), cudaMemcpyDeviceToHost));

	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * 6] = accu_host->h[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * 6] = hessian[c + r * 6];

	memcpy(nabla, accu_host->g, noPara * sizeof(float));
	f = (accu_host->numPoints > 100) ? sqrt(accu_host->f) / accu_host->numPoints : 1e5f;

	return std::make_pair(host_matches, accu_host->numPoints);
}

// device functions

template<bool shortIteration, bool rotationOnly>
__device__ void depthTrackerOneLevel_g_rt_device_main(ITMDepthTracker_CUDA::AccuCell *accu, Vector4f* matches, float *depth, Matrix4f approxInvPose, Vector4f *pointsMap,
	Vector4f *normalsMap, Vector4f sceneIntrinsics, Vector2i sceneImageSize, Matrix4f scenePose, Vector4f viewIntrinsics, Vector2i viewImageSize,
	float distThresh)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	int locId_local = threadIdx.x + threadIdx.y * blockDim.x;

	__shared__ float dim_shared1[256];
	__shared__ float dim_shared2[256];
	__shared__ float dim_shared3[256];
	__shared__ bool should_prefix;

	should_prefix = false;
	__syncthreads();

	const int noPara = shortIteration ? 3 : 6;
	const int noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;
	float A[noPara]; float b;
	Vector4f match;
	bool isValidPoint = false;

	if (x < viewImageSize.x && y < viewImageSize.y)
	{
		match = computePerPointGH_Depth_Ab<shortIteration, rotationOnly>(A, b, x, y, depth[x + y * viewImageSize.x],
			viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh);
		matches[x + y * viewImageSize.x] = match;
		if (match.w != 0.0) {
			isValidPoint = true;
		}
		if (isValidPoint) should_prefix = true;
	}

	if (!isValidPoint) {
		for (int i = 0; i < noPara; i++) A[i] = 0.0f;
		b = 0.0f;
	}

	__syncthreads();

	if (!should_prefix) return;
	
	{ //reduction for noValidPoints
		dim_shared1[locId_local] = isValidPoint;
		__syncthreads();

		if (locId_local < 128) dim_shared1[locId_local] += dim_shared1[locId_local + 128];
		__syncthreads();
		if (locId_local < 64) dim_shared1[locId_local] += dim_shared1[locId_local + 64];
		__syncthreads();

		if (locId_local < 32) warpReduce(dim_shared1, locId_local);

		if (locId_local == 0) atomicAdd(&(accu->numPoints), (int)dim_shared1[locId_local]);
	}

	{ //reduction for energy function value
		dim_shared1[locId_local] = b*b;
		__syncthreads();

		if (locId_local < 128) dim_shared1[locId_local] += dim_shared1[locId_local + 128];
		__syncthreads();
		if (locId_local < 64) dim_shared1[locId_local] += dim_shared1[locId_local + 64];
		__syncthreads();

		if (locId_local < 32) warpReduce(dim_shared1, locId_local);

		if (locId_local == 0) atomicAdd(&(accu->f), dim_shared1[locId_local]);
	}

	__syncthreads();

	//reduction for nabla
	for (unsigned char paraId = 0; paraId < noPara; paraId+=3)
	{
		dim_shared1[locId_local] = b*A[paraId+0];
		dim_shared2[locId_local] = b*A[paraId+1];
		dim_shared3[locId_local] = b*A[paraId+2];
		__syncthreads();

		if (locId_local < 128) {
			dim_shared1[locId_local] += dim_shared1[locId_local + 128];
			dim_shared2[locId_local] += dim_shared2[locId_local + 128];
			dim_shared3[locId_local] += dim_shared3[locId_local + 128];
		}
		__syncthreads();
		if (locId_local < 64) {
			dim_shared1[locId_local] += dim_shared1[locId_local + 64];
			dim_shared2[locId_local] += dim_shared2[locId_local + 64];
			dim_shared3[locId_local] += dim_shared3[locId_local + 64];
		}
		__syncthreads();

		if (locId_local < 32) {
			warpReduce(dim_shared1, locId_local);
			warpReduce(dim_shared2, locId_local);
			warpReduce(dim_shared3, locId_local);
		}
		__syncthreads();

		if (locId_local == 0) {
			atomicAdd(&(accu->g[paraId+0]), dim_shared1[0]);
			atomicAdd(&(accu->g[paraId+1]), dim_shared2[0]);
			atomicAdd(&(accu->g[paraId+2]), dim_shared3[0]);
		}
	}

	__syncthreads();

	float localHessian[noParaSQ];
#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))
#pragma unroll
#endif
	for (unsigned char r = 0, counter = 0; r < noPara; r++)
	{
#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))
#pragma unroll
#endif
		for (int c = 0; c <= r; c++, counter++) localHessian[counter] = A[r] * A[c];
	}

	//reduction for hessian
	for (unsigned char paraId = 0; paraId < noParaSQ; paraId+=3)
	{
		dim_shared1[locId_local] = localHessian[paraId+0];
		dim_shared2[locId_local] = localHessian[paraId+1];
		dim_shared3[locId_local] = localHessian[paraId+2];
		__syncthreads();

		if (locId_local < 128) {
			dim_shared1[locId_local] += dim_shared1[locId_local + 128];
			dim_shared2[locId_local] += dim_shared2[locId_local + 128];
			dim_shared3[locId_local] += dim_shared3[locId_local + 128];
		}
		__syncthreads();
		if (locId_local < 64) {
			dim_shared1[locId_local] += dim_shared1[locId_local + 64];
			dim_shared2[locId_local] += dim_shared2[locId_local + 64];
			dim_shared3[locId_local] += dim_shared3[locId_local + 64];
		}
		__syncthreads();

		if (locId_local < 32) {
			warpReduce(dim_shared1, locId_local);
			warpReduce(dim_shared2, locId_local);
			warpReduce(dim_shared3, locId_local);
		}
		__syncthreads();

		if (locId_local == 0) {
			atomicAdd(&(accu->h[paraId+0]), dim_shared1[0]);
			atomicAdd(&(accu->h[paraId+1]), dim_shared2[0]);
			atomicAdd(&(accu->h[paraId+2]), dim_shared3[0]);
		}
	}
}

template<bool shortIteration, bool rotationOnly>
__global__ void depthTrackerOneLevel_g_rt_device(ITMDepthTracker_KernelParameters para)
{
	depthTrackerOneLevel_g_rt_device_main<shortIteration, rotationOnly>(para.accu, para.matches, para.depth, para.approxInvPose, para.pointsMap, para.normalsMap, para.sceneIntrinsics, para.sceneImageSize, para.scenePose, para.viewIntrinsics, para.viewImageSize, para.distThresh);
}

