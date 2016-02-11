// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMDepthTracker_CPU.h"
#include "../../DeviceAgnostic/ITMDepthTracker.h"

using namespace ITMLib::Engine;

ITMDepthTracker_CPU::ITMDepthTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel,
	float distThresh, float terminationThreshold, bool visualize_icp, const ITMLowLevelEngine *lowLevelEngine) :ITMDepthTracker(imgSize, trackingRegime, noHierarchyLevels,
	noICPRunTillLevel, distThresh, terminationThreshold, visualize_icp, lowLevelEngine, MEMORYDEVICE_CPU) { }

ITMDepthTracker_CPU::~ITMDepthTracker_CPU(void) { }

std::pair<Vector4f*, int> ITMDepthTracker_CPU::ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{

	Vector4f *pointsMap = sceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalsMap = sceneHierarchyLevel->normalsMap->GetData(MEMORYDEVICE_CPU);
	Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;
	Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;
	Vector4f* matches;

	float *depth = viewHierarchyLevel->depth->GetData(MEMORYDEVICE_CPU);
	Vector4f viewIntrinsics = viewHierarchyLevel->intrinsics;
	Vector2i viewImageSize = viewHierarchyLevel->depth->noDims;

	if (iterationType == TRACKER_ITERATION_NONE) {
	  return std::make_pair(matches, 0);
	}

	bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);

	float sumHessian[6 * 6], sumNabla[6], sumF; int noValidPoints;
	int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

	noValidPoints = 0; sumF = 0.0f;
	memset(sumHessian, 0, sizeof(float) * noParaSQ);
	memset(sumNabla, 0, sizeof(float) * noPara);
	memset(matches, 0, sizeof(Vector4f) * viewImageSize.height * viewImageSize.width);

	for (int y = 0; y < viewImageSize.y; y++) for (int x = 0; x < viewImageSize.x; x++)
	{
		float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;

		for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

		Vector4f match;
		bool isValidPoint;

//		std::cout << "NNS size check: " << nns->cloud.cols() << std::endl;
		switch (iterationType)
		{
		case TRACKER_ITERATION_ROTATION:
		  match = computePerPointGH_Depth_NN<true, true>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], viewImageSize,
				viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh[levelId], nns);
			break;
		case TRACKER_ITERATION_TRANSLATION:
		  match = computePerPointGH_Depth_NN<true, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], viewImageSize,
				viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh[levelId], nns);
			break;
		case TRACKER_ITERATION_BOTH:
		  match = computePerPointGH_Depth_NN<false, false>(localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], viewImageSize,
				viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh[levelId], nns);
			break;
		default:
		  match.w = 0.0;
			isValidPoint = false;
			break;
		}

		matches[x + y * viewImageSize.x] = match;

		if (match.w != 0.0) {
		  isValidPoint = true;
		}

		if (isValidPoint)
		{
			noValidPoints++; sumF += localF;
			for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];
			for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];
		}
	}

	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * 6] = sumHessian[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * 6] = hessian[c + r * 6];
	
	memcpy(nabla, sumNabla, noPara * sizeof(float));
	f = (noValidPoints > 100) ? sqrt(sumF) / noValidPoints : 1e5f;

	return std::make_pair(matches, noValidPoints);
}







template<bool shortIteration, bool rotationOnly>
Vector4f ITMDepthTracker_CPU::computePerPointGH_Depth_Ab_NN(THREADPTR(float) *A, THREADPTR(float) &b,
  const THREADPTR(int) & x, const THREADPTR(int) & y,
  const CONSTPTR(float) &depth, const CONSTPTR(Vector2i) & viewImageSize, const CONSTPTR(Vector4f) & viewIntrinsics, const CONSTPTR(Vector2i) & sceneImageSize,
  const CONSTPTR(Vector4f) & sceneIntrinsics, const CONSTPTR(Matrix4f) & approxInvPose, const CONSTPTR(Matrix4f) & scenePose, const CONSTPTR(Vector4f) *pointsMap,
  const CONSTPTR(Vector4f) *normalsMap, float distThresh, const boost::shared_ptr<Nabo::NNSearchF>& nns = Nabo::NNSearchF::createKDTreeLinearHeap(Eigen::MatrixXf(1, 1)))
{

  Vector4f tmp3Dpoint, tmp3Dpoint_reproj; Vector3f ptDiff;
  Vector4f curr3Dpoint, corr3Dnormal; Vector2f tmp2Dpoint;

  if (depth <= 1e-8f) {
    curr3Dpoint.w = 0.0;
    return curr3Dpoint;
  }

  tmp3Dpoint.x = depth * ((float(x) - viewIntrinsics.z) / viewIntrinsics.x);
  tmp3Dpoint.y = depth * ((float(y) - viewIntrinsics.w) / viewIntrinsics.y);
  tmp3Dpoint.z = depth;
  tmp3Dpoint.w = 1.0f;

  // transform to previous frame coordinates
  tmp3Dpoint = approxInvPose * tmp3Dpoint;
  tmp3Dpoint.w = 1.0f;
  tmp3Dpoint_reproj = scenePose * tmp3Dpoint;
  if (tmp3Dpoint_reproj.z <= 0.0f) {
    curr3Dpoint.w = 0.0;
    return curr3Dpoint;
  }

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
      std::cout << "NN YAYYYYYYYYY " << index.coeff(0) << " " << dists2.coeff(0) << std::endl;
    } else {
      std::cout << "NN ZEROSSSSSSS " << index.coeff(0) << " " << dists2.coeff(0) << std::endl;
      curr3Dpoint.w = 0.0;
      return curr3Dpoint;
    }
  } else {
    std::cout << "NOT Using Libnabo NN" << std::endl;
    tmp2Dpoint.x = sceneIntrinsics.x * tmp3Dpoint_reproj.x / tmp3Dpoint_reproj.z + sceneIntrinsics.z;
    tmp2Dpoint.y = sceneIntrinsics.y * tmp3Dpoint_reproj.y / tmp3Dpoint_reproj.z + sceneIntrinsics.w;

    if (!((tmp2Dpoint.x >= 0.0f) && (tmp2Dpoint.x <= sceneImageSize.x - 2) && (tmp2Dpoint.y >= 0.0f) && (tmp2Dpoint.y <= sceneImageSize.y - 2))) {
      curr3Dpoint.w = 0.0;
      return curr3Dpoint;
    }

    curr3Dpoint = interpolateBilinear_withHoles(pointsMap, tmp2Dpoint, sceneImageSize);
  }

  if (curr3Dpoint.w < 0.0f) {
    curr3Dpoint.w = 0.0;
    return curr3Dpoint;
  }

  ptDiff.x = curr3Dpoint.x - tmp3Dpoint.x;
  ptDiff.y = curr3Dpoint.y - tmp3Dpoint.y;
  ptDiff.z = curr3Dpoint.z - tmp3Dpoint.z;
  float dist = ptDiff.x * ptDiff.x + ptDiff.y * ptDiff.y + ptDiff.z * ptDiff.z;

  if (dist > distThresh) {
    curr3Dpoint.w = 0.0;
    return curr3Dpoint;
  }

  corr3Dnormal = interpolateBilinear_withHoles(normalsMap, tmp2Dpoint, sceneImageSize);
//  if (corr3Dnormal.w < 0.0f) return false;

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

  return curr3Dpoint;
}

template<bool shortIteration, bool rotationOnly>
Vector4f ITMDepthTracker_CPU::computePerPointGH_Depth_NN(THREADPTR(float) *localNabla, THREADPTR(float) *localHessian, THREADPTR(float) &localF,
  const THREADPTR(int) & x, const THREADPTR(int) & y,
  const CONSTPTR(float) &depth, const CONSTPTR(Vector2i) & viewImageSize, const CONSTPTR(Vector4f) & viewIntrinsics, const CONSTPTR(Vector2i) & sceneImageSize,
  const CONSTPTR(Vector4f) & sceneIntrinsics, const CONSTPTR(Matrix4f) & approxInvPose, const CONSTPTR(Matrix4f) & scenePose, const CONSTPTR(Vector4f) *pointsMap,
  const CONSTPTR(Vector4f) *normalsMap, float distThresh, const boost::shared_ptr<Nabo::NNSearchF>& nns = Nabo::NNSearchF::createKDTreeLinearHeap(Eigen::MatrixXf(1, 1)))
{
  const int noPara = shortIteration ? 3 : 6;
  float A[noPara];
  float b;

  Vector4f match = computePerPointGH_Depth_Ab_NN<shortIteration,rotationOnly>(A, b, x, y, depth, viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh, nns);

  if (match.w == 0.0) return match;

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

  return match;
}
