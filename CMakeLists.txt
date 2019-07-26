cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(kiva_rotate_center SHARED kiva_rotate_center.cc)
target_link_libraries(kiva_rotate_center ${GAZEBO_LIBRARIES})

add_executable(main main.cc)
target_link_libraries(main ${GAZEBO_LIBRARIES} pthread)

add_executable(mov mov.cc)

if (${gazebo_VERSION_MAJOR} LESS 6)
  # These two
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(mov ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(mov ${GAZEBO_LIBRARIES})
endif()

add_executable(movx movx.cc)

if (${gazebo_VERSION_MAJOR} LESS 6)
  # These two
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(movx ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(movx ${GAZEBO_LIBRARIES})
endif()

add_executable(movy movy.cc)

if (${gazebo_VERSION_MAJOR} LESS 6)
  # These two
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(movy ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(movy ${GAZEBO_LIBRARIES})
endif()

add_executable(rot rot.cc)

if (${gazebo_VERSION_MAJOR} LESS 6)
  # These two
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(rot ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(rot ${GAZEBO_LIBRARIES})
endif()

add_executable(lift lift.cc)

if (${gazebo_VERSION_MAJOR} LESS 6)
  # These two
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(lift ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(lift ${GAZEBO_LIBRARIES})
endif()