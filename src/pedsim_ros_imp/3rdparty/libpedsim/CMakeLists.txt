cmake_minimum_required(VERSION 2.8.3)
project(pedsim)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


#------------------ Configuration ------------------#
option(SHALL_DEBUG "Enable debug features" OFF)
option(SHALL_PROFILE "Enable the code profiling feature" OFF)
option(CMAKE_VERBOSE_MAKEFILE "Full compiler output" ON)


#----------------- Compiler Flags ------------------#
add_definitions(-Wall -Wunused -std=c++0x -pipe)

#debugging
if(SHALL_DEBUG)
  message("Debugging activated")
  add_definitions(-O0 -DDEBUG -ggdb -g3 -rdynamic)
else(SHALL_DEBUG)
  message("Debugging deactivated")
  add_definitions(-Os)
endif(SHALL_DEBUG)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

catkin_package(
  CATKIN_DEPENDS  roscpp
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/
  LIBRARIES pedsim
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/pedsim/
  ${roscpp_INCLUDE_DIRS}
)

set(SOURCES
  src/ped_agent.cpp
  src/ped_angle.cpp
  src/ped_obstacle.cpp
  src/ped_scene.cpp
  src/ped_tree.cpp
  src/ped_vector.cpp
  src/ped_waypoint.cpp
)

add_library(pedsim ${SOURCES})

target_link_libraries(pedsim
  ${BOOST_LIBRARIES}
)
