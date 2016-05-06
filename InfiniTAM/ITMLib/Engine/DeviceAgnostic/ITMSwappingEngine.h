// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"

_CPU_AND_GPU_CODE_ inline int forwardProjectPoint(const THREADPTR(Vector4f) &pt_model,
  const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) &projParams_d, const CONSTPTR(Vector2i) &imgSize)
{
  Vector4f pt_camera;
  Vector2f pt_image;

  pt_camera = M_d * pt_model;

  if (pt_camera.z < 1e-10f) return -1;

  pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
  pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;

  if (pt_image.x < 0 || pt_image.x > imgSize.x-1 || pt_image.y < 0 || pt_image.y > imgSize.y-1) return -1;

  return (int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x;
}


template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void combineVoxelDepthInformation(const CONSTPTR(TVoxel) & src, DEVICEPTR(TVoxel) & dst, int maxW, const double update_time)
{
	int newW = dst.w_depth;
	int oldW = src.w_depth;
	float newF = TVoxel::SDF_valueToFloat(dst.sdf);
	float oldF = TVoxel::SDF_valueToFloat(src.sdf);

	if (oldW == 0) return;

	newF = oldW * oldF + newW * newF;
	newW = oldW + newW;
	newF /= newW;
	newW = MIN(newW, maxW);

	dst.last_update_time = update_time/1000.0;
	dst.w_depth = newW;
	dst.sdf = TVoxel::SDF_floatToValue(newF);
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void combineVoxelColorInformation(const CONSTPTR(TVoxel) & src, DEVICEPTR(TVoxel) & dst, int maxW)
{
	int newW = dst.w_color;
	int oldW = src.w_color;
	Vector3f newC = dst.clr.toFloat() / 255.0f;
	Vector3f oldC = src.clr.toFloat() / 255.0f;

	if (oldW == 0) return;

	newC = oldC * (float)oldW + newC * (float)newW;
	newW = oldW + newW;
	newC /= (float)newW;
	newW = MIN(newW, maxW);

	dst.clr = TO_UCHAR3(newC * 255.0f);
	dst.w_color = (uchar)newW;
}


template<bool hasColor,class TVoxel> struct CombineVoxelInformation;

template<class TVoxel>
struct CombineVoxelInformation<false,TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(const CONSTPTR(TVoxel) & src, DEVICEPTR(TVoxel) & dst, int maxW, const double update_time)
	{
		combineVoxelDepthInformation(src, dst, maxW, update_time);
	}
};

template<class TVoxel>
struct CombineVoxelInformation<true,TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(const CONSTPTR(TVoxel) & src, DEVICEPTR(TVoxel) & dst, int maxW, const double update_time)
	{
		combineVoxelDepthInformation(src, dst, maxW, update_time);
		combineVoxelColorInformation(src, dst, maxW);
	}
};

