CMAKE_MINIMUM_REQUIRED(VERSION 3.1) 
PROJECT(POSITION_ANALYZE)

SET(version 2023)


message(STATUS "platform: " ${platform})
message(STATUS "system: " ${system})
message(STATUS "release: " ${release})

SET(CMAKE_BUILD_TYPE "Release")
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(PROJ)
find_package(Threads REQUIRED)

INCLUDE_DIRECTORIES(
  ${PROJECT_SOURCE_DIR}/include/
  ${EIGEN3_INCLUDE_DIR}
  ${SOPHUS_INCLUDE_DIR}
  /usr/local/include/bfl
  /home/yuxuanliang/catkin_ws/PROJ-master/build/install/include

)

link_directories(
  /usr/local/lib
)

AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src/ SOURCE_DIR)
ADD_EXECUTABLE(POSITION_ANALYZE ${SOURCE_DIR})

TARGET_LINK_LIBRARIES(POSITION_ANALYZE PRIVATE Sophus orocos-bfl pthread ${PROJ_LIBRARIES})