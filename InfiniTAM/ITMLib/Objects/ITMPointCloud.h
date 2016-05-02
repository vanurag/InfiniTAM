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
			uint noTotalPoints, noTotalActivePoints, noTotalInactivePoints;

			ORUtils::Image<Vector4f> *locations, *colours, *active_locations, *inactive_locations;

			explicit ITMPointCloud(Vector2i imgSize, MemoryDeviceType memoryType)
			{
				this->noTotalPoints = 0;
				this->noTotalActivePoints = 0;
				this->noTotalInactivePoints = 0;

				locations = new ORUtils::Image<Vector4f>(imgSize, memoryType);
				active_locations = new ORUtils::Image<Vector4f>(imgSize, memoryType);
				inactive_locations = new ORUtils::Image<Vector4f>(imgSize, memoryType);
				colours = new ORUtils::Image<Vector4f>(imgSize, memoryType);
			}

			void UpdateHostFromDevice()
			{
				this->locations->UpdateHostFromDevice();
				this->active_locations->UpdateHostFromDevice();
				this->inactive_locations->UpdateHostFromDevice();
				this->colours->UpdateHostFromDevice();
			}

			void UpdateDeviceFromHost()
			{
				this->locations->UpdateDeviceFromHost();
				this->active_locations->UpdateDeviceFromHost();
				this->inactive_locations->UpdateDeviceFromHost();
				this->colours->UpdateDeviceFromHost();
			}

			~ITMPointCloud()
			{
				delete locations;
				delete active_locations;
				delete inactive_locations;
				delete colours;
			}

			// Suppress the default copy constructor and assignment operator
			ITMPointCloud(const ITMPointCloud&);
			ITMPointCloud& operator=(const ITMPointCloud&);
		};
	}
}
