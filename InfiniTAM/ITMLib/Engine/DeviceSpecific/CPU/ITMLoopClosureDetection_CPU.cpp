/*
 * ITMLoopClosureDetection_CPU.cpp
 *
 *  Created on: May 13, 2016
 *      Author: anurag
 */

// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMLoopClosureDetection_CPU.h"
#include "../../DeviceAgnostic/ITMDepthTracker.h"

using namespace ITMLib::Engine;

ITMLoopClosureDetection_CPU::ITMLoopClosureDetection_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels, int noICPRunTillLevel,
  float distThresh, float terminationThreshold, ITMLibSettings::DepthTrackerType tracker_type, bool visualize_icp, const ITMLowLevelEngine *lowLevelEngine) :ITMLoopClosureDetection(imgSize, trackingRegime, noHierarchyLevels,
  noICPRunTillLevel, distThresh, terminationThreshold, tracker_type, visualize_icp, lowLevelEngine, MEMORYDEVICE_CPU) { }

ITMLoopClosureDetection_CPU::~ITMLoopClosureDetection_CPU(void) { }

std::pair<Vector4f*, int> ITMLoopClosureDetection_CPU::ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose)
{

  Vector4f *pointsMap = sceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CPU);
  Vector4f *normalsMap = sceneHierarchyLevel->normalsMap->GetData(MEMORYDEVICE_CPU);
  Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;
  Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;
  Vector4f* matches;

  Vector4f *inactivePointsMap = inactiveSceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CPU);
  Vector4f inactiveSceneIntrinsics = inactiveSceneHierarchyLevel->intrinsics;
  Vector2i inactiveSceneImageSize = inactiveSceneHierarchyLevel->pointsMap->noDims;

  if (iterationType == TRACKER_ITERATION_NONE) {
    return std::make_pair(matches, 0);
  }

  bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);

  float sumHessian[6 * 6], sumNabla[6], sumF; int noValidPoints;
  int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

  noValidPoints = 0; sumF = 0.0f;
  memset(sumHessian, 0, sizeof(float) * noParaSQ);
  memset(sumNabla, 0, sizeof(float) * noPara);
  matches = (Vector4f*) malloc(sizeof(Vector4f) * inactiveSceneImageSize.height * inactiveSceneImageSize.width);
  memset(matches, 0, sizeof(Vector4f) * inactiveSceneImageSize.height * inactiveSceneImageSize.width);

  for (int y = 0; y < inactiveSceneImageSize.y; y++) for (int x = 0; x < inactiveSceneImageSize.x; x++)
  {
    float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;

    for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
    for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

    Vector4f match;
    bool isValidPoint;

//    std::cout << "NNS size check: " << nns->cloud.cols() << std::endl;
    switch (iterationType)
    {
    case TRACKER_ITERATION_ROTATION:
      match = computePerPointGH_Depth<true, true>(localNabla, localHessian, localF, x, y, inactivePointsMap[x + y * inactiveSceneImageSize.x], inactiveSceneImageSize,
          inactiveSceneIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh[levelId]);
      break;
    case TRACKER_ITERATION_TRANSLATION:
      match = computePerPointGH_Depth<true, false>(localNabla, localHessian, localF, x, y, inactivePointsMap[x + y * inactiveSceneImageSize.x], inactiveSceneImageSize,
          inactiveSceneIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh[levelId]);
      break;
    case TRACKER_ITERATION_BOTH:
      match = computePerPointGH_Depth<false, false>(localNabla, localHessian, localF, x, y, inactivePointsMap[x + y * inactiveSceneImageSize.x], inactiveSceneImageSize,
          inactiveSceneIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh[levelId]);
      break;
    default:
      match.w = 0.0;
      isValidPoint = false;
      break;
    }

    matches[x + y * inactiveSceneImageSize.x] = match;

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
