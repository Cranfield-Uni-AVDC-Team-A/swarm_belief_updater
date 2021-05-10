cmake_minimum_required(VERSION 2.8.3)
project(swarm_belief_updater_process)

## ================= Compiler-specific C++11 activation =================

if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  if (NOT (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7))
    message(FATAL_ERROR "${PROJECT_NAME} requires g++ 4.7 or greater.")
  endif ()
elseif ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
else ()
  message(FATAL_ERROR "Your C++ compiler does not support C++11.")
endif ()

add_definitions(-std=c++11)

## ================= Set project variables ==================

set(SWARM_BELIEF_UPDATER_PROCESS_SOURCE_DIR src/source)

set(SWARM_BELIEF_UPDATER_PROCESS_INCLUDE_DIR src/include)

set(SWARM_BELIEF_UPDATER_PROCESS_SOURCE_FILES
  ${SWARM_BELIEF_UPDATER_PROCESS_SOURCE_DIR}/swarm_belief_updater_process.cpp
  ${SWARM_BELIEF_UPDATER_PROCESS_SOURCE_DIR}/swarm_belief_updater_process_main.cpp
)

set(SWARM_BELIEF_UPDATER_PROCESS_HEADER_FILES
  ${SWARM_BELIEF_UPDATER_PROCESS_INCLUDE_DIR}/swarm_belief_updater_process.h
)

## ================= Find catkin macros and libraries =================

find_package(catkin REQUIRED COMPONENTS
  roscpp
  droneMsgsROS
  std_msgs
  aerostack_msgs
  belief_manager_msgs
)
## ================= Catkin specific configuration =================

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  droneMsgsROS
  aerostack_msgs
  belief_manager_msgs
  DEPENDS yaml-cpp
)

## ================= Build =================

include_directories(${SWARM_BELIEF_UPDATER_PROCESS_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

add_library(
  swarm_belief_updater_process_lib
  ${SWARM_BELIEF_UPDATER_PROCESS_SOURCE_FILES}
  ${SWARM_BELIEF_UPDATER_PROCESS_HEADER_FILES}
)
add_dependencies(swarm_belief_updater_process_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_belief_updater_process_lib ${catkin_LIBRARIES})

add_executable(
  swarm_belief_updater_process_main
  ${SWARM_BELIEF_UPDATER_PROCESS_SOURCE_DIR}/swarm_belief_updater_process_main.cpp
)
add_dependencies(swarm_belief_updater_process_main ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_belief_updater_process_main swarm_belief_updater_process_lib)
target_link_libraries(swarm_belief_updater_process_main ${catkin_LIBRARIES})
target_link_libraries(swarm_belief_updater_process_main yaml-cpp)

