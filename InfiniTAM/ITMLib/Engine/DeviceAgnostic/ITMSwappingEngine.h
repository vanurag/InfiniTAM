// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMLibDefines.h"

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

	dst.last_update_time = static_cast<short int>(update_time/1000.0);
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

