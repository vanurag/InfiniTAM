find_package(freenect2)

include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

IF(OPENNI_FOUND)
  include_directories(${OpenNI_INCLUDE_DIR})
ELSE(OPENNI_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_OpenNI)
ENDIF(OPENNI_FOUND)

IF(OPENCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
ELSE(OPENCV_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_OpenCV)
ENDIF(OPENCV_FOUND)

IF(MSKINECTAPI_FOUND)
  include_directories(${MSKINECTAPI_INCLUDE_DIR})
ELSE(MSKINECTAPI_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_Kinect2SDK)
ENDIF(MSKINECTAPI_FOUND)

IF(freenect2_FOUND)
  include_directories(${freenect2_INCLUDE_DIRS})
ELSE(freenect2_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_freenect2)
ENDIF(freenect2_FOUND)

IF(DEFINED libuvc_INCLUDE_DIRS)
  include_directories(${libuvc_INCLUDE_DIRS})
  add_definitions(-DCOMPILE_WITH_LibUVC)
ENDIF()

add_library(Engine
ImageSourceEngine.cpp
ImageSourceEngine.h
IMUSourceEngine.cpp
IMUSourceEngine.h
ROSIMUSourceEngine.cpp
ROSIMUSourceEngine.h
Kinect2Engine.cpp
Kinect2Engine.h
RealsenseEngine.cpp
RealsenseEngine.h
VISensorEngine.cpp
VISensorEngine.h
OpenNIEngine.cpp
OpenNIEngine.h
LibUVCEngine.cpp
LibUVCEngine.h
UIEngine.cpp
UIEngine.h
CLIEngine.cpp
CLIEngine.h
)

target_link_libraries(Engine ${GLUT_LIBRARIES})
target_link_libraries(Engine ${OPENGL_LIBRARIES})
target_link_libraries(Engine ${catkin_LIBRARIES})
target_link_libraries(Engine ITMLib)
IF(WITH_CUDA)
  target_link_libraries(Engine ${CUDA_LIBRARY})
ENDIF()
IF(OPENNI_FOUND)
  target_link_libraries(Engine ${OpenNI_LIBRARY})
ENDIF(OPENNI_FOUND)
IF(OPENCV_FOUND)
  target_link_libraries(Engine ${OpenCV_LIBS})
ENDIF(OPENCV_FOUND)
IF(freenect2_FOUND)
	target_link_libraries(Engine ${freenect2_LIBRARY})
ENDIF()

IF(DEFINED libuvc_LIBRARIES)
  target_link_libraries(Engine ${libuvc_LIBRARIES})
ENDIF()
