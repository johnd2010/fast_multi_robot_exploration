cmake_minimum_required(VERSION 3.13)
project(pose_utils)

find_package(catkin REQUIRED COMPONENTS
  #armadillo
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_utils
  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

find_package(Armadillo REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
    include
    )

add_library(pose_utils 
   ${ARMADILLO_LIBRARIES}
   src/pose_utils.cpp)
