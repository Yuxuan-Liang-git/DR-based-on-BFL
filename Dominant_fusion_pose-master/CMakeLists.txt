cmake_minimum_required(VERSION 2.8)
project(FusionPose)

SET(version 2020)
# SET(platform x86_64)
# SET(system ubuntu_18_04)

execute_process(COMMAND uname -m OUTPUT_VARIABLE platform OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND lsb_release -is OUTPUT_VARIABLE system OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND lsb_release -rs OUTPUT_VARIABLE release OUTPUT_STRIP_TRAILING_WHITESPACE)

message(STATUS "platform: " ${platform})
message(STATUS "system: " ${system})
message(STATUS "release: " ${release})

# SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "Release")

string(TIMESTAMP VERSION %Y%m%d%H%M )
configure_file (
  "${PROJECT_SOURCE_DIR}/include/version.h.in"
  "${PROJECT_SOURCE_DIR}/include/version.h"
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11")

# find_package(g2o REQUIRED)
#find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(Eigen3 3.1.0 REQUIRED)
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        /usr/local/include/bfl
        #${SOPHUS_INCLUDE_DIR}
        /usr/local/include/eigen3/)

# set(SOURCE_FILES src/nonlinearanalyticconditionalgaussianodo.cpp src/PoseFuse.cpp src/utm.cpp src/datum.cpp)
# add_executable(FusionPose main.cpp ${SOURCE_FILES})
# target_link_libraries(FusionPose Sophus orocos-bfl lcm pthread ${OpenCV_LIBS})

AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src SOURCE_DIR)
ADD_EXECUTABLE(FusionPose_${version}_${system}_${release}_${platform} ${SOURCE_DIR})
TARGET_LINK_LIBRARIES(FusionPose_${version}_${system}_${release}_${platform}
  CommonApi Config
  gflags glog
	Sophus orocos-bfl lcm zcm pthread gsl gslcblas xml2
	${OpenCV_LIBS})