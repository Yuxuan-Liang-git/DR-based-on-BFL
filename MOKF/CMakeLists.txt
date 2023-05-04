CMAKE_MINIMUM_REQUIRED(VERSION 3.1) 
PROJECT(POSITION_ANALYZE)

SET(version 2023)


message(STATUS "platform: " ${platform})
message(STATUS "system: " ${system})
message(STATUS "release: " ${release})

SET(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3)
INCLUDE_DIRECTORIES(
  ${PROJECT_SOURCE_DIR}/include/
  ${EIGEN3_INCLUDE_DIR}

)

link_directories(


)

AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src/ SOURCE_DIR)
ADD_EXECUTABLE(POSITION_ANALYZE ${SOURCE_DIR})



