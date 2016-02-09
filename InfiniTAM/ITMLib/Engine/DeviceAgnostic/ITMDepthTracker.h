// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"
#include "ITMPixelUtils.h"

template<bool shortIteration, bool rotationOnly>
_CPU_AND_GPU_CODE_ inline bool computePerPointGH_Depth_Ab(THREADPTR(float) *A, THREADPTR(float) &b,
	const THREADPTR(int) & x, const THREADPTR(int) & y,
	const CONSTPTR(float) &depth, const CONSTPTR(Vector2i) & viewImageSize, const CONSTPTR(Vector4f) & viewIntrinsics, const CONSTPTR(Vector2i) & sceneImageSize,
	const CONSTPTR(Vector4f) & sceneIntrinsics, const CONSTPTR(Matrix4f) & approxInvPose, const CONSTPTR(Matrix4f) & scenePose, const CONSTPTR(Vector4f) *pointsMap,
	const CONSTPTR(Vector4f) *normalsMap, float distThresh, const boost::shared_ptr<Nabo::NNSearchF>& nns = Nabo::NNSearchF::createKDTreeLinearHeap(Eigen::MatrixXf(1, 1)))
{
	if (depth <= 1e-8f) return false; //check if valid -- != 0.0f

	Vector4f tmp3Dpoint, tmp3Dpoint_reproj; Vector3f ptDiff;
	Vector4f curr3Dpoint, corr3Dnormal; Vector2f tmp2Dpoint;

	tmp3Dpoint.x = depth * ((float(x) - viewIntrinsics.z) / viewIntrinsics.x);
	tmp3Dpoint.y = depth * ((float(y) - viewIntrinsics.w) / viewIntrinsics.y);
	tmp3Dpoint.z = depth;
	tmp3Dpoint.w = 1.0f;
    
	// transform to previous frame coordinates
	tmp3Dpoint = approxInvPose * tmp3Dpoint;
	tmp3Dpoint.w = 1.0f;
	tmp3Dpoint_reproj = scenePose * tmp3Dpoint;
  if (tmp3Dpoint_reproj.z <= 0.0f) return false;

	// project into previous rendered image -> point match pairs for ICP
	if (nns->cloud.cols() > 1) {
	  std::cout << "Using Libnabo NN " << nns->cloud.cols() << std::endl << std::endl;
	  std::cout << "Sample cloud: " << std::endl;
	  for (int c = 0, i = 0; c < 3 && i < nns->cloud.cols(); ++i) {
	    if (nns->cloud.col(i).sum() != 0) {
	      c += 1;
	      std::cout << "Col " << i << ": " << nns->cloud.col(i) << std::endl;
	    }
	  }
	  std::cout << std::endl;

	  Eigen::Vector3f query(tmp3Dpoint_reproj.x, tmp3Dpoint_reproj.y, tmp3Dpoint_reproj.z);
	  Eigen::VectorXi index(1);  // 1 nearest neighbour index required
	  Eigen::VectorXf dists2(1);  // dist to nearest neighbour
	  nns->knn(query, index, dists2, 1, 3.16, Nabo::NearestNeighbourSearch<float>::ALLOW_SELF_MATCH);

	  curr3Dpoint.x = nns->cloud(0, index.coeff(0));
	  curr3Dpoint.y = nns->cloud(1, index.coeff(0));
	  curr3Dpoint.z = nns->cloud(2, index.coeff(0));
	  curr3Dpoint.w = 1.0;

	  if (curr3Dpoint.x != 0.0 || curr3Dpoint.y != 0.0 || curr3Dpoint.z != 0.0) {
	    std::cout << curr3Dpoint.x << ", " << curr3Dpoint.y << ", " << curr3Dpoint.z << ", " << std::endl;
	  } else {
	    std::cout << "NN ZEROSSSSSSS " << index.coeff(0) << " " << dists2.coeff(0) << std::endl;
	    return false;
	  }
	} else {
	  std::cout << "NOT Using Libnabo NN" << std::endl;
    tmp2Dpoint.x = sceneIntrinsics.x * tmp3Dpoint_reproj.x / tmp3Dpoint_reproj.z + sceneIntrinsics.z;
    tmp2Dpoint.y = sceneIntrinsics.y * tmp3Dpoint_reproj.y / tmp3Dpoint_reproj.z + sceneIntrinsics.w;

    if (!((tmp2Dpoint.x >= 0.0f) && (tmp2Dpoint.x <= sceneImageSize.x - 2) && (tmp2Dpoint.y >= 0.0f) && (tmp2Dpoint.y <= sceneImageSize.y - 2)))
      return false;

    curr3Dpoint = interpolateBilinear_withHoles(pointsMap, tmp2Dpoint, sceneImageSize);
	}

	if (curr3Dpoint.w < 0.0f) return false;

	ptDiff.x = curr3Dpoint.x - tmp3Dpoint.x;
	ptDiff.y = curr3Dpoint.y - tmp3Dpoint.y;
	ptDiff.z = curr3Dpoint.z - tmp3Dpoint.z;
	float dist = ptDiff.x * ptDiff.x + ptDiff.y * ptDiff.y + ptDiff.z * ptDiff.z;

	if (dist > distThresh) return false;

	corr3Dnormal = interpolateBilinear_withHoles(normalsMap, tmp2Dpoint, sceneImageSize);
//	if (corr3Dnormal.w < 0.0f) return false;

	b = corr3Dnormal.x * ptDiff.x + corr3Dnormal.y * ptDiff.y + corr3Dnormal.z * ptDiff.z;

	// TODO check whether normal matches normal from image, done in the original paper, but does not seem to be required
	if (shortIteration)
	{
		if (rotationOnly)
		{
			A[0] = +tmp3Dpoint.z * corr3Dnormal.y - tmp3Dpoint.y * corr3Dnormal.z;
			A[1] = -tmp3Dpoint.z * corr3Dnormal.x + tmp3Dpoint.x * corr3Dnormal.z;
			A[2] = +tmp3Dpoint.y * corr3Dnormal.x - tmp3Dpoint.x * corr3Dnormal.y;
		}
		else { A[0] = corr3Dnormal.x; A[1] = corr3Dnormal.y; A[2] = corr3Dnormal.z; }
	}
	else
	{
		A[0] = +tmp3Dpoint.z * corr3Dnormal.y - tmp3Dpoint.y * corr3Dnormal.z;
		A[1] = -tmp3Dpoint.z * corr3Dnormal.x + tmp3Dpoint.x * corr3Dnormal.z;
		A[2] = +tmp3Dpoint.y * corr3Dnormal.x - tmp3Dpoint.x * corr3Dnormal.y;
		A[!shortIteration ? 3 : 0] = corr3Dnormal.x; A[!shortIteration ? 4 : 1] = corr3Dnormal.y; A[!shortIteration ? 5 : 2] = corr3Dnormal.z;
	}

	return true;
}

template<bool shortIteration, bool rotationOnly>
_CPU_AND_GPU_CODE_ inline bool computePerPointGH_Depth(THREADPTR(float) *localNabla, THREADPTR(float) *localHessian, THREADPTR(float) &localF,
	const THREADPTR(int) & x, const THREADPTR(int) & y,
	const CONSTPTR(float) &depth, const CONSTPTR(Vector2i) & viewImageSize, const CONSTPTR(Vector4f) & viewIntrinsics, const CONSTPTR(Vector2i) & sceneImageSize,
	const CONSTPTR(Vector4f) & sceneIntrinsics, const CONSTPTR(Matrix4f) & approxInvPose, const CONSTPTR(Matrix4f) & scenePose, const CONSTPTR(Vector4f) *pointsMap,
	const CONSTPTR(Vector4f) *normalsMap, float distThresh, const boost::shared_ptr<Nabo::NNSearchF>& nns = Nabo::NNSearchF::createKDTreeLinearHeap(Eigen::MatrixXf(1, 1)))
{
	const int noPara = shortIteration ? 3 : 6;
	float A[noPara];
	float b;

	bool ret = computePerPointGH_Depth_Ab<shortIteration,rotationOnly>(A, b, x, y, depth, viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh, nns);

	if (!ret) return false;

	localF = b * b;

#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))
#pragma unroll
#endif
	for (int r = 0, counter = 0; r < noPara; r++)
	{
		localNabla[r] = b * A[r];
#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))
#pragma unroll
#endif
		for (int c = 0; c <= r; c++, counter++) localHessian[counter] = A[r] * A[c];
	}

	return true;
}

