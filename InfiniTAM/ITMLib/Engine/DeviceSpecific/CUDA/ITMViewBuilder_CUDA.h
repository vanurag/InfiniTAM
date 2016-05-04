// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMViewBuilder.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMViewBuilder_CUDA : public ITMViewBuilder
		{
		public:
			void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics, 
				Vector2f disparityCalibParams);
			void ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, Vector2f depthCalibParams);

			void DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in);
			void ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic);
			void ComputeColorForDepthImage(ITMUChar4Image *rgb_out, const ITMExtrinsics rgb_to_depth, const ITMFloatImage *depth, const ITMIntrinsics intrinsics_depth, const ITMIntrinsics intrinsics_rgb, const ITMUChar4Image *rgb);

			void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise = false);
			void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage);

			void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement);

			void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMOdometryMeasurement *odomMeasurement);

			ITMViewBuilder_CUDA(const ITMRGBDCalib *calib);
			~ITMViewBuilder_CUDA(void);
		};
	}
}
