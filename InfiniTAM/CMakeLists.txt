cmake_minimum_required(VERSION 2.8)
project(InfiniTAM)

find_package(catkin REQUIRED COMPONENTS 
	           roscpp 
	           message_generation 
	           std_msgs
	           cv_bridge
	           image_transport)
	            
find_package(Boost REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

OPTION(ENABLE_CXX11 "Enable C++11 support" ON)

IF(ENABLE_CXX11)
  INCLUDE(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  IF(COMPILER_SUPPORTS_CXX11)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  ELSEIF(COMPILER_SUPPORTS_CXX0X)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  ELSE()
    MESSAGE(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  ENDIF()
ENDIF(ENABLE_CXX11)

macro(CANONIFY_BOOL var)
  if (${var})
    set(${var} TRUE)
  else()
    set(${var} FALSE)
  endif()
endmacro()

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

find_package(CUDA QUIET)
CANONIFY_BOOL(CUDA_FOUND)
message(STATUS "CUDA found: " ${CUDA_FOUND})

find_package(OpenNI2)
CANONIFY_BOOL(OPENNI2_FOUND)
message(STATUS "OpenNI2 found: " ${OPENNI2_FOUND})

find_package(OpenCV CONFIG COMPONENTS core features2d viz REQUIRED
             HINTS /home/anurag/kinfu_ws/devel/)
CANONIFY_BOOL(OpenCV_FOUND)
message(STATUS "OpenCV2 found: " ${OpenCV_FOUND})
#CANONIFY_BOOL(opencv_core_FOUND)  
#message(STATUS "OpenCV2 VIZ found: " ${opencv_core_FOUND})      

find_package(libuvc QUIET)
CANONIFY_BOOL(libuvc_FOUND)
message(STATUS "libuvc found: ${libuvc_FOUND}")

find_package(PNG QUIET)
CANONIFY_BOOL(PNG_FOUND)
message(STATUS "libpng found: ${PNG_FOUND}")

#--------------------
# DEPENDENCY: nabo
#--------------------
find_package(libnabo REQUIRED)
include_directories(${libnabo_INCLUDE_DIRS})
message("-- libnabo found, version ${libnabo_VERSION}")

OPTION(WITH_CUDA "Build with CUDA support?" ${CUDA_FOUND})
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)

IF(MSVC_IDE)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-DUSING_CMAKE=1)
ELSE(MSVC_IDE)
  set(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  set(CMAKE_CXX_FLAGS "-fPIC -O3 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
  #set(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
ENDIF(MSVC_IDE)

if(APPLE)
  set(CUDA_HOST_COMPILER /usr/bin/clang)
endif(APPLE)

IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
ENDIF()

add_subdirectory(ORUtils)
add_subdirectory(ITMLib)
add_subdirectory(Utils)
add_subdirectory(Engine)

IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

add_executable(InfiniTAM_cli InfiniTAM_cli.cpp)
target_link_libraries(InfiniTAM_cli Engine)
target_link_libraries(InfiniTAM_cli Utils)
add_executable(InfiniTAM InfiniTAM.cpp)
target_link_libraries(InfiniTAM Engine)
target_link_libraries(InfiniTAM Utils)

