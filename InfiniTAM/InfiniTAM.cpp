// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include <cstdlib>
#include <ros/ros.h>

#include "Engine/UIEngine.h"
#include "Engine/ImageSourceEngine.h"
#include "Engine/ROSBagSourceEngine.h"
#include "Engine/ROSImageSourceEngine.h"
#include "Engine/ROSIMUSourceEngine.h"
#include "Engine/ROSOdometrySourceEngine.h"
#include "Engine/OpenNIEngine.h"
#include "Engine/Kinect2Engine.h"
#include "Engine/VISensorEngine.h"
#include "Engine/LibUVCEngine.h"
#include "Engine/RealSenseEngine.h"

using namespace InfiniTAM::Engine;

/** Create a default source of depth images from a list of command line
    arguments. Typically, @para arg1 would identify the calibration file to
    use, @para arg2 the colour images, @para arg3 the depth images and
    @para arg4 the IMU images. If images are omitted, some live sources will
    be tried.
*/
static void CreateDefaultImageSource(
    ImageSourceEngine* & imageSource, IMUSourceEngine* & imuSource, OdometrySourceEngine* & odomSource,
    const char *arg1, const char *arg2, const char *arg3, const char *arg4, const char *arg5, const char *arg6)
{
	const char *calibFile = arg1;
	const char *source = arg2;
	const char *imu_source = arg3;
	const char *odom_source = arg3;
	const char *meta1 = arg4;
	const char *meta2 = arg5;
	const char *meta3 = arg6;

	printf("using calibration file: %s\n", calibFile);

	if (meta2 != NULL && source == std::string("any"))
	{
		printf("using rgb images: %s\nusing depth images: %s\n", meta1, meta2);
		if (imu_source == NULL)
		{
			imageSource = new ImageFileReader(calibFile, meta1, meta2);
		}
		else
		{
			printf("using imu data: %s\n", imu_source);
			imageSource = new RawFileReader(calibFile, meta1, meta2, Vector2i(320, 240), 0.5f);
			imuSource = new IMUSourceEngine(imu_source);
		}
	}

	if (imageSource == NULL && source == std::string("any"))
	{
		printf("trying OpenNI device: %s\n", (meta1==NULL)?"<OpenNI default device>":meta1);
		imageSource = new OpenNIEngine(calibFile, meta1);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}
	if (imageSource == NULL && source == std::string("any"))
	{
		printf("trying UVC device\n");
		imageSource = new LibUVCEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}
	if (imageSource == NULL && source == std::string("any"))
  {
    printf("trying MS Kinect 2 device\n");
    imageSource = new Kinect2Engine(calibFile);
    if (imageSource->getDepthImageSize().x == 0)
    {
      //delete imageSource;
      imageSource = NULL;
    }
  }
	if (imageSource == NULL && source == std::string("any"))
	{
		printf("trying RealSense device (only Windows)\n");
		imageSource = new RealSenseEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0)
		{
			delete imageSource;
			imageSource = NULL;
		}
	}


	if (imageSource == NULL &&
      (source == std::string("kinect_ros") || source == std::string("kinect_ros+imu") ||
       source == std::string("kinect_ros+odom") || source == std::string("kinect_ros+strict_odom") ||
       source == std::string("realsense_ros") || source == std::string("realsense_ros+imu") ||
       source == std::string("realsense_ros+odom") || source == std::string("realsense_ros+strict_odom")))
  {
    printf("trying ROS image source\n");
    if (source == std::string("kinect_ros") || source == std::string("kinect_ros+imu") ||
        source == std::string("kinect_ros+odom") || source == std::string("kinect_ros+strict_odom")) {
      imageSource = new ROSImageSourceEngine(
          calibFile, meta1, meta2, Vector2i(1920, 1080), Vector2i(512, 424));
    } else if (source == std::string("realsense_ros") || source == std::string("realsense_ros+imu") ||
               source == std::string("realsense_ros+odom") || source == std::string("realsense_ros+strict_odom")) {
      imageSource = new ROSImageSourceEngine(
          calibFile, meta1, meta2, Vector2i(640, 480), Vector2i(480, 360));
    }
    if (imageSource->getDepthImageSize().x == 0)
    {
      //delete imageSource;
      imageSource = NULL;
    }
    if (source == std::string("kinect_ros+imu") || source == std::string("realsense_ros+imu")) {
      if (imu_source != NULL) {
        printf("using IMU ROS topic: %s\n", imu_source);
        imuSource = new ROSIMUSourceEngine(imu_source);
      } else {
        printf("IMU source not provided! aborting.");
        return;
      }
    }
    if (source == std::string("kinect_ros+odom") || source == std::string("kinect_ros+strict_odom") ||
        source == std::string("realsense_ros+odom") || source == std::string("realsense_ros+strict_odom")) {
      if (odom_source != NULL) {
        printf("using Odometry ROS topic: %s\n", odom_source);
        odomSource = new ROSOdometrySourceEngine(odom_source);
      } else {
        printf("Odometry source not provided! aborting.");
        return;
      }
    }
  }
	if (imageSource == NULL &&
	    (source == std::string("realsense") || source == std::string("realsense+imu") ||
       source == std::string("realsense+odom") || source == std::string("realsense+strict_odom")))
  {
    printf("trying Intel Realsense device\n");
//    imageSource = new RealsenseEngine(calibFile);
    imageSource = new ROSImageSourceEngine(
        calibFile, meta1, meta2, Vector2i(640, 480), Vector2i(480, 360));
    if (imageSource->getDepthImageSize().x == 0)
    {
      delete imageSource;
      imageSource = NULL;
    }
    if (source == std::string("realsense+imu")) {
      if (imu_source != NULL) {
        printf("using IMU ROS topic: %s\n", imu_source);
        imuSource = new ROSIMUSourceEngine(imu_source);
      } else {
        printf("IMU source not provided! aborting.");
        return;
      }
    }
    if (source == std::string("realsense+odom") || source == std::string("realsense+strict_odom")) {
      if (odom_source != NULL) {
        printf("using Odometry ROS topic: %s\n", odom_source);
        odomSource = new ROSOdometrySourceEngine(odom_source);
      } else {
        printf("Odometry source not provided! aborting.");
        return;
      }
    }
  }
	if (imageSource == NULL && source == std::string("vi-sensor"))
  {
    printf("trying Skybotix VI-Sensor\n");
    imageSource = new VISensorEngine(calibFile);
    if (imageSource->getDepthImageSize().x == 0)
    {
      delete imageSource;
      imageSource = NULL;
    }
    if (imu_source != NULL) {
      printf("using IMU ROS topic: %s\n", imu_source);
      imuSource = new ROSIMUSourceEngine(imu_source);
    }
  }
	if (imageSource == NULL &&
	    (source == std::string("kinect") || source == std::string("kinect+imu") ||
       source == std::string("kinect+odom") || source == std::string("kinect+strict_odom")))
  {
    printf("trying MS Kinect 2 device\n");
    imageSource = new Kinect2Engine(calibFile);
    if (imageSource->getDepthImageSize().x == 0)
    {
      //delete imageSource;
      imageSource = NULL;
    }
    if (source == std::string("kinect+imu")) {
      if (imu_source != NULL) {
        printf("using IMU ROS topic: %s\n", imu_source);
        imuSource = new ROSIMUSourceEngine(imu_source);
      } else {
        printf("IMU source not provided! aborting.");
        return;
      }
    }
    if (source == std::string("kinect+odom") || source == std::string("kinect+strict_odom")) {
      if (odom_source != NULL) {
        printf("using Odometry ROS topic: %s\n", odom_source);
        odomSource = new ROSOdometrySourceEngine(odom_source);
      } else {
        printf("Odometry source not provided! aborting.");
        return;
      }
    }
  }
	if (imageSource == NULL &&
      (source == std::string("kinect_rosbag") || source == std::string("kinect_rosbag+imu") ||
       source == std::string("kinect_rosbag+odom") || source == std::string("kinect_rosbag+strict_odom")))
  {
    printf("trying Kinect ROSBAG image source\n");
    if (source == std::string("kinect_rosbag")) {
      ROSBagSourceEngine rosbag_source(calibFile, meta1, meta2, meta3,
                                       Vector2i(1920, 1080), Vector2i(512, 424));
      imageSource = rosbag_source.rosbag_image_source_engine;
    } else if (source == std::string("kinect_rosbag+imu")) {
      if (imu_source != NULL) {
        printf("using IMU ROS topic: %s\n", imu_source);
        ROSBagSourceEngine rosbag_source(calibFile, meta1, meta2, meta3, imu_source,
                                         Vector2i(1920, 1080), Vector2i(512, 424), "imu");
        imageSource = rosbag_source.rosbag_image_source_engine;
        imuSource = rosbag_source.rosbag_imu_source_engine;
      } else {
        printf("IMU source not provided! aborting.");
        return;
      }
    } else if (source == std::string("kinect_rosbag+odom") || source == std::string("kinect_rosbag+strict_odom")) {
      if (odom_source != NULL) {
        printf("using Odometry ROS topic: %s\n", odom_source);
        ROSBagSourceEngine rosbag_source(calibFile, meta1, meta2, meta3, imu_source,
                                         Vector2i(1920, 1080), Vector2i(512, 424), "odom");
        imageSource = rosbag_source.rosbag_image_source_engine;
        odomSource = rosbag_source.rosbag_odometry_source_engine;
      } else {
        printf("Odometry source not provided! aborting.");
        return;
      }
    }
    if (imageSource->getDepthImageSize().x == 0)
    {
      //delete imageSource;
      imageSource = NULL;
    }
  }
	if (imageSource == NULL &&
      (source == std::string("realsense_rosbag") || source == std::string("realsense_rosbag+imu") ||
       source == std::string("realsense_rosbag+odom") || source == std::string("realsense_rosbag+strict_odom")))
  {
	  if (source == std::string("realsense_rosbag")) {
	    ROSBagSourceEngine rosbag_source(calibFile, meta1, meta2, meta3,
	                                     Vector2i(640, 480), Vector2i(480, 360));
      imageSource = rosbag_source.rosbag_image_source_engine;
    } else if (source == std::string("realsense_rosbag+imu")) {
      if (imu_source != NULL) {
        printf("using IMU ROS topic: %s\n", imu_source);
        ROSBagSourceEngine rosbag_source(calibFile, meta1, meta2, meta3, imu_source,
                                         Vector2i(640, 480), Vector2i(480, 360), "imu");
        imageSource = rosbag_source.rosbag_image_source_engine;
        imuSource = rosbag_source.rosbag_imu_source_engine;
      } else {
        printf("IMU source not provided! aborting.");
        return;
      }
    } else if (source == std::string("realsense_rosbag+odom") || source == std::string("realsense_rosbag+strict_odom")) {
      if (odom_source != NULL) {
        printf("using Odometry ROS topic: %s\n", odom_source);
        ROSBagSourceEngine rosbag_source(calibFile, meta1, meta2, meta3, imu_source,
                                         Vector2i(640, 480), Vector2i(480, 360), "odom");
        imageSource = rosbag_source.rosbag_image_source_engine;
        odomSource = rosbag_source.rosbag_odometry_source_engine;
      } else {
        printf("Odometry source not provided! aborting.");
        return;
      }
    }
    if (imageSource->getDepthImageSize().x == 0)
    {
      //delete imageSource;
      imageSource = NULL;
    }
  }
	if (imageSource == NULL && meta2 != NULL && source == std::string("offline"))
  {
    printf("using rgb images: %s\nusing depth images: %s\n", meta1, meta2);
    if (imu_source == NULL)
    {
      imageSource = new ImageFileReader(calibFile, meta1, meta2);
    }
    else
    {
      printf("using imu data: %s\n", imu_source);
      imageSource = new RawFileReader(calibFile, meta1, meta2, Vector2i(320, 240), 0.5f);
      imuSource = new IMUSourceEngine(imu_source);
    }
  }

	// this is a hack to ensure backwards compatibility in certain configurations
	if (imageSource == NULL) return;
	if (imageSource->calib.disparityCalib.params == Vector2f(0.0f, 0.0f))
	{
		imageSource->calib.disparityCalib.type = ITMDisparityCalib::TRAFO_AFFINE;
		imageSource->calib.disparityCalib.params = Vector2f(1.0f/1000.0f, 0.0f);
	}
}

int main(int argc, char** argv)
try
{

  if (argc < 3) {
    printf("usage: %s [<calibfile>] [<source>] [optional:<imusource>] [optional:metadata]\n"
           "  <calibfile>    : path to a file containing intrinsic calibration parameters\n"
           "  <source>       : source input device 'any'/'realsense'/'vi-sensor'\n"
           "  <posesource>   : ROS IMU/Odometry topic/file containing transformations\n"
           "  <metadata>     : either one argument to specify OpenNI device ID\n"
           "                   or two arguments specifying rgb and depth file masks\n"
           "\n"
           "examples:\n"
           "  %s ./Files/Teddy/calib.txt any ./Files/Teddy/Frames/%%04i.ppm ./Files/Teddy/Frames/%%04i.pgm\n"
           "  %s ./Files/Teddy/calib.txt realsense\n\n", argv[0], argv[0], argv[0]);
    return -1;
  }

  ros::Time::init();
  ros::init(argc, argv, "infinitam_node");
  ROS_INFO("Starting infinitam_node with node name %s", ros::this_node::getName().c_str());

	const char *arg1 = "";    // calib
	const char *arg2 = NULL;  // source device
	const char *arg3 = NULL;  // IMU/Odometry source
	const char *arg4 = NULL;  // metadata-1
	const char *arg5 = NULL;  // metadata-2
	const char *arg6 = NULL;  // metadata-3

	int arg = 1;
	do {
		if (argv[arg] != NULL) arg1 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) arg2 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) arg3 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) arg4 = argv[arg]; else break;
		++arg;
    if (argv[arg] != NULL) arg5 = argv[arg]; else break;
    ++arg;
    if (argv[arg] != NULL) arg6 = argv[arg]; else break;
	} while (false);

	printf("initialising ...\n");
	ImageSourceEngine *imageSource = NULL;
	IMUSourceEngine *imuSource = NULL;
	OdometrySourceEngine *odomSource = NULL;

	CreateDefaultImageSource(imageSource, imuSource, odomSource, arg1, arg2, arg3, arg4, arg5, arg6);
	if (imageSource==NULL)
	{
		std::cout << "failed to open any image stream" << std::endl;
		return -1;
	}
	if (imuSource==NULL && odomSource == NULL)
  {
    std::cout << "Proceeding without a pose source" << std::endl;
  }

	ITMLibSettings *internalSettings = new ITMLibSettings();
//	internalSettings->visualizeICP = true;
	internalSettings->depthTrackerType = ITMLibSettings::TRACKER_ITM;
	if (arg2 == std::string("kinect") || arg2 == std::string("kinect_ros") ||
	    arg2 == std::string("kinect_rosbag")) { // Kinect2
	  internalSettings->sceneParams.viewFrustum_min = 0.5f;
	  internalSettings->sceneParams.viewFrustum_max = 8.0f;
	} else if (arg2 == std::string("kinect+imu") || arg2 == std::string("kinect_ros+imu") ||
	           arg2 == std::string("kinect_rosbag+imu")) { // Kinect2
    internalSettings->sceneParams.viewFrustum_min = 0.5f;
    internalSettings->sceneParams.viewFrustum_max = 8.0f;
    internalSettings->trackerType = ITMLibSettings::TRACKER_IMU;
	} else if (arg2 == std::string("kinect+odom") || arg2 == std::string("kinect_ros+odom") ||
	           arg2 == std::string("kinect_rosbag+odom")) { // Kinect2
    internalSettings->sceneParams.viewFrustum_min = 0.5f;
    internalSettings->sceneParams.viewFrustum_max = 8.0f;
    internalSettings->trackerType = ITMLibSettings::TRACKER_ODOMETRY;
	} else if (arg2 == std::string("kinect+strict_odom") || arg2 == std::string("kinect_ros+strict_odom") ||
	           arg2 == std::string("kinect_rosbag+strict_odom")) { // Kinect2
    internalSettings->sceneParams.viewFrustum_min = 0.5f;
    internalSettings->sceneParams.viewFrustum_max = 8.0f;
    internalSettings->trackerType = ITMLibSettings::TRACKER_STRICT_ODOMETRY;
	} else if (arg2 == std::string("realsense") || arg2 == std::string("realsense_ros") ||
	           arg2 == std::string("realsense_rosbag")) { // R200
	  internalSettings->sceneParams.viewFrustum_min = 0.5f;
	  internalSettings->sceneParams.viewFrustum_max = 2.0f; // 4.0f
	} else if (arg2 == std::string("realsense+imu") || arg2 == std::string("realsense_ros+imu") ||
	           arg2 == std::string("realsense_rosbag+imu")) { // R200
    internalSettings->sceneParams.viewFrustum_min = 0.5f;
    internalSettings->sceneParams.viewFrustum_max = 2.0f; // 4.0f
    internalSettings->trackerType = ITMLibSettings::TRACKER_IMU;
	} else if (arg2 == std::string("realsense+odom") || arg2 == std::string("realsense_ros+odom") ||
	           arg2 == std::string("realsense_rosbag+odom")) { // R200
    internalSettings->sceneParams.viewFrustum_min = 0.5f;
    internalSettings->sceneParams.viewFrustum_max = 2.0f; // 4.0f
    internalSettings->trackerType = ITMLibSettings::TRACKER_ODOMETRY;
	} else if (arg2 == std::string("realsense+strict_odom") || arg2 == std::string("realsense_ros+strict_odom") ||
	           arg2 == std::string("realsense_rosbag+strict_odom")) { // R200
    internalSettings->sceneParams.viewFrustum_min = 0.5f;
    internalSettings->sceneParams.viewFrustum_max = 2.0f; // 4.0f
    internalSettings->trackerType = ITMLibSettings::TRACKER_STRICT_ODOMETRY;
	} else if (arg2 == std::string("vi-sensor")) {
	  internalSettings->sceneParams.viewFrustum_min = 0.0f;
    internalSettings->sceneParams.viewFrustum_max = 20.0f;
	}
	std::cout << "Setting viewFrustum to the range: [ "
      << internalSettings->sceneParams.viewFrustum_min << ", "
      << internalSettings->sceneParams.viewFrustum_max << " ]" << std::endl;
	ITMMainEngine *mainEngine = new ITMMainEngine(internalSettings, &imageSource->calib, imageSource->getRGBImageSize(), imageSource->getDepthImageSize());

	UIEngine::Instance()->Initialise(argc, argv, imageSource, imuSource, odomSource, mainEngine, "./Files/Out", internalSettings);
	UIEngine::Instance()->Run();
	UIEngine::Instance()->Shutdown();

  printf("Exiting IoHandler!");
  ros::shutdown();

	delete mainEngine;
	delete internalSettings;
	delete imageSource;
	if (imuSource != NULL) delete imuSource;
	if (odomSource != NULL) delete odomSource;
	return 0;
}
catch(std::exception& e)
{
	std::cerr << e.what() << '\n';
	return EXIT_FAILURE;
}

