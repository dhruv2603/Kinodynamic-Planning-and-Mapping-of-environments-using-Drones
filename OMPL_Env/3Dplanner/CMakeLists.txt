cmake_minimum_required(VERSION 3.10)

project(3Dplanner)

set(CMAKE_CXX_STANDARD 11)
find_package(ompl REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

add_executable(drone3D CollisionChecking.cpp drone_3d.cpp)
target_link_libraries(drone3D ${OMPL_LIBRARIES})
