// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../../ORUtils/Image.h"

#include <stdlib.h>

namespace ITMLib
{
	namespace Objects
	{
		class ITMPointCloud
		{
		public:
			uint noTotalPoints, noTotalInactivePoints;

			ORUtils::Image<Vector4f> *locations, *colours, *inactive_locations, *remapped_inactive_locations, *inactive_colours;

			explicit ITMPointCloud(Vector2i imgSize, MemoryDeviceType memoryType)
			{
				this->noTotalPoints = 0;
				this->noTotalInactivePoints = 0;

				locations = new ORUtils::Image<Vector4f>(imgSize, memoryType);
				inactive_locations = new ORUtils::Image<Vector4f>(imgSize, memoryType);
				remapped_inactive_locations = new ORUtils::Image<Vector4f>(imgSize, memoryType);
				colours = new ORUtils::Image<Vector4f>(imgSize, memoryType);
				inactive_colours = new ORUtils::Image<Vector4f>(imgSize, memoryType);
			}

			void UpdateHostFromDevice()
			{
				this->locations->UpdateHostFromDevice();
				this->inactive_locations->UpdateHostFromDevice();
				this->remapped_inactive_locations->UpdateHostFromDevice();
				this->colours->UpdateHostFromDevice();
				this->inactive_colours->UpdateHostFromDevice();
			}

			void UpdateDeviceFromHost()
			{
				this->locations->UpdateDeviceFromHost();
				this->inactive_locations->UpdateDeviceFromHost();
				this->remapped_inactive_locations->UpdateDeviceFromHost();
				this->colours->UpdateDeviceFromHost();
				this->inactive_colours->UpdateDeviceFromHost();
			}

			~ITMPointCloud()
			{
				delete locations;
				delete inactive_locations;
				delete remapped_inactive_locations;
				delete colours;
				delete inactive_colours;
			}

			// Suppress the default copy constructor and assignment operator
			ITMPointCloud(const ITMPointCloud&);
			ITMPointCloud& operator=(const ITMPointCloud&);
		};
	}
}
