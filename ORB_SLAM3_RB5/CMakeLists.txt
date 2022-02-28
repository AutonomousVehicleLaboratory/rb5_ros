cmake_minimum_required(VERSION 2.8.12)
project(ORB_SLAM3_RB5)

set(ORB_SLAM3_SOURCE_DIR "path/to/ORB_SLAM3_RB5/")

find_package(catkin REQUIRED COMPONENTS
   roscpp
   tf
   sensor_msgs
   image_transport
   cv_bridge
)

catkin_package(
   CATKIN_DEPENDS
   roscpp
   tf
   sensor_msgs
   image_transport
   cv_bridge
)
# rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${ORB_SLAM3_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.2 QUIET)
if(NOT OpenCV_FOUND)
	message(FATAL_ERROR "OpenCV > 4.4 not found.")
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${ORB_SLAM3_SOURCE_DIR}/
${ORB_SLAM3_SOURCE_DIR}/include
${ORB_SLAM3_SOURCE_DIR}/include/CameraModels
${ORB_SLAM3_SOURCE_DIR}/Thirdparty/Sophus
${Pangolin_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${ORB_SLAM3_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${ORB_SLAM3_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${ORB_SLAM3_SOURCE_DIR}/lib/libORB_SLAM3.so
-lboost_system
)

# Node for monocular camera
add_executable(Mono
src/ros_mono.cc
)

target_link_libraries(Mono
${LIBS}
${catkin_LIBRARIES}
)

# Node for monocular camera (Augmented Reality Demo)
add_executable(MonoAR
src/AR/ros_mono_ar.cc
src/AR/ViewerAR.h
src/AR/ViewerAR.cc
)

target_link_libraries(MonoAR
${LIBS}
${catkin_LIBRARIES}
)

# Node for stereo camera
add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo
${LIBS}
${catkin_LIBRARIES}
)

# Node for RGB-D camera
add_executable(RGBD
src/ros_rgbd.cc
)

target_link_libraries(RGBD
${LIBS}
${catkin_LIBRARIES}
)

# Node for monocular-inertial camera
add_executable(Mono_Inertial
src/ros_mono_inertial.cc
)

target_link_libraries(Mono_Inertial
${LIBS}
${catkin_LIBRARIES}
)

# Node for stereo-inertial camera
add_executable(Stereo_Inertial
src/ros_stereo_inertial.cc
)

target_link_libraries(Stereo_Inertial
${LIBS}
${catkin_LIBRARIES}
)
