cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#add_library(kiva_rotate SHARED kiva_rotate_center.cc)
#target_link_libraries(kiva_rotate ${GAZEBO_LIBRARIES})
add_library(kiva_rotate_center SHARED kiva_rotate_center.cc)
target_link_libraries(kiva_rotate_center ${GAZEBO_LIBRARIES})
#add_library(kiva_rotate_with_load SHARED kiva_rotate_with_load.cc)
#target_link_libraries(kiva_rotate_with_load ${GAZEBO_LIBRARIES})
