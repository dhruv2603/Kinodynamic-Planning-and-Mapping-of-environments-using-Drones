cmake_minimum_required(VERSION 3.10)

project(2Dplanner)

set(CMAKE_CXX_STANDARD 11)
find_package(ompl REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

add_executable(drone2D CollisionChecking.cpp drone.cpp)
target_link_libraries(drone2D ${OMPL_LIBRARIES})
