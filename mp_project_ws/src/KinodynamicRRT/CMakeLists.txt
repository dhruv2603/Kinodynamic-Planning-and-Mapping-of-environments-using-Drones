cmake_minimum_required(VERSION 3.0.2)
project(KinodynamicRRT)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_msgs
  octomap_msgs
  octomap_ros
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf
  std_msgs
  visualization_msgs
)

find_package(dynamicEDT3D)
find_package(OpenCV REQUIRED)
find_package(PkgConfig)
find_package(ompl REQUIRED)
pkg_search_module(Eigen3 REQUIRED eigen3)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories( ${DYNAMICEDT3D_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${OMPL_INCLUDE_DIRS})

add_executable(drone3D src/drone_3d.cpp)
target_link_libraries(drone3D ${catkin_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES} ${OMPL_LIBRARIES})

