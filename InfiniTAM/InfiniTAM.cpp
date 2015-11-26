// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include <cstdlib>
#include <ros/ros.h>

#include "Engine/UIEngine.h"
#include "Engine/ImageSourceEngine.h"
#include "Engine/ROSIMUSourceEngine.h"
#include "Engine/OpenNIEngine.h"
#include "Engine/Kinect2Engine.h"
#include "Engine/RealsenseEngine.h"
#include "Engine/VISensorEngine.h"
#include "Engine/LibUVCEngine.h"
#include "Engine/ROSIMUSourceEngine.h"

using namespace InfiniTAM::Engine;

/** Create a default source of depth images from a list of command line
    arguments. Typically, @para arg1 would identify the calibration file to
    use, @para arg2 the colour images, @para arg3 the depth images and
    @para arg4 the IMU images. If images are omitted, some live sources will
    be tried.
*/
static void CreateDefaultImageSource(
    ImageSourceEngine* & imageSource, IMUSourceEngine* & imuSource, const char *arg1,
    const char *arg2, const char *arg3, const char *arg4, const char *arg5)
{
	const char *calibFile = arg1;
	const char *source = arg2;
	const char *imu_source = arg3;
	const char *filename1 = arg4;
	const char *filename2 = arg5;

	printf("using calibration file: %s\n", calibFile);

	if (filename2 != NULL && source == std::string("any"))
	{
		printf("using rgb images: %s\nusing depth images: %s\n", filename1, filename2);
		if (imu_source == NULL)
		{
			imageSource = new ImageFileReader(calibFile, filename1, filename2);
		}
		else
		{
			printf("using imu data: %s\n", imu_source);
			imageSource = new RawFileReader(calibFile, filename1, filename2, Vector2i(320, 240), 0.5f);
			imuSource = new IMUSourceEngine(imu_source);
		}
	}

	if (imageSource == NULL && source == std::string("any"))
	{
		printf("trying OpenNI device: %s\n", (filename1==NULL)?"<OpenNI default device>":filename1);
		imageSource = new OpenNIEngine(calibFile, filename1);
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

	if (imageSource == NULL && source == std::string("realsense"))
  {
    printf("trying Intel Realsense device\n");
    imageSource = new RealsenseEngine(calibFile);
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
	if (imageSource == NULL && source == std::string("kinect"))
  {
    printf("trying MS Kinect 2 device\n");
    imageSource = new Kinect2Engine(calibFile);
    if (imageSource->getDepthImageSize().x == 0)
    {
      //delete imageSource;
      imageSource = NULL;
    }
  }
	if (imageSource == NULL && filename2 != NULL && source == std::string("offline"))
  {
    printf("using rgb images: %s\nusing depth images: %s\n", filename1, filename2);
    if (imu_source == NULL)
    {
      imageSource = new ImageFileReader(calibFile, filename1, filename2);
    }
    else
    {
      printf("using imu data: %s\n", imu_source);
      imageSource = new RawFileReader(calibFile, filename1, filename2, Vector2i(320, 240), 0.5f);
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
           "  <calibfile>   : path to a file containing intrinsic calibration parameters\n"
           "  <source>      : source input device 'any'/'realsense'/'vi-sensor'\n"
           "  <imusource>   : ROS IMU topic/file containing transformations\n"
           "  <metadata>    : either one argument to specify OpenNI device ID\n"
           "                  or two arguments specifying rgb and depth file masks\n"
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
	const char *arg3 = NULL;  // IMU source
	const char *arg4 = NULL;  // metadata-1
	const char *arg5 = NULL;  // metadata-2

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
	} while (false);

	printf("initialising ...\n");
	ImageSourceEngine *imageSource = NULL;
	IMUSourceEngine *imuSource = NULL;

	CreateDefaultImageSource(imageSource, imuSource, arg1, arg2, arg3, arg4, arg5);
	if (imageSource==NULL)
	{
		std::cout << "failed to open any image stream" << std::endl;
		return -1;
	}

	ITMLibSettings *internalSettings = new ITMLibSettings();
	if (arg2 == std::string("kinect")) { // Kinect2
	  internalSettings->sceneParams.viewFrustum_min = 0.5f;
	  internalSettings->sceneParams.viewFrustum_max = 8.0f;
	} else if (arg2 == std::string("realsense")) { // R200
	  internalSettings->sceneParams.viewFrustum_min = 0.5f;
	  internalSettings->sceneParams.viewFrustum_max = 4.0f;
	} else if (arg2 == std::string("vi-sensor")) {
	  internalSettings->sceneParams.viewFrustum_min = 0.2f;
    internalSettings->sceneParams.viewFrustum_max = 20.0f;
	}
	std::cout << "Setting viewFrustum to the range: [ "
      << internalSettings->sceneParams.viewFrustum_min << ", "
      << internalSettings->sceneParams.viewFrustum_max << " ]" << std::endl;
	ITMMainEngine *mainEngine = new ITMMainEngine(internalSettings, &imageSource->calib, imageSource->getRGBImageSize(), imageSource->getDepthImageSize());

	UIEngine::Instance()->Initialise(argc, argv, imageSource, imuSource, mainEngine, "./Files/Out", internalSettings);
	UIEngine::Instance()->Run();
	UIEngine::Instance()->Shutdown();

  printf("Exiting IoHandler!");
  ros::shutdown();

	delete mainEngine;
	delete internalSettings;
	delete imageSource;
	if (imuSource != NULL) delete imuSource;
	return 0;
}
catch(std::exception& e)
{
	std::cerr << e.what() << '\n';
	return EXIT_FAILURE;
}

