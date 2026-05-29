cmake_minimum_required(VERSION 3.5)
project(go1control)

option(DEBUG_ASAN "Enable ASAN debug option" OFF)
set(HARDWARE_TARGET "" CACHE STRING "Hardware target to build: simulation, go1, robot")
set_property(CACHE HARDWARE_TARGET PROPERTY STRINGS simulation go1 robot)

if(NOT HARDWARE_TARGET MATCHES "^(simulation|go1|robot)$")
  message(FATAL_ERROR "HARDWARE_TARGET must be one of: simulation, go1, robot (got '${HARDWARE_TARGET}')")
endif()
message(STATUS "Hardware target: ${HARDWARE_TARGET}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "install prefix" FORCE)
endif()

## RTROBOT_DIR may be provided as an environment variable or as a CMake cache variable.
## Prefer an explicit CMake configure-time setting. If not provided, default to $HOME/rtrobot
if(NOT DEFINED RTROBOT_DIR)
  if(DEFINED ENV{RTROBOT_DIR})
    set(RTROBOT_DIR "$ENV{RTROBOT_DIR}" CACHE PATH "Path to rtrobot directory")
  else()
    set(RTROBOT_DIR "$ENV{HOME}/rtrobot" CACHE PATH "Path to rtrobot directory")
  endif()
else()
  # ensure RTROBOT_DIR is in the cache so it can be modified by the user in cmake-gui or via -D
  set(RTROBOT_DIR "${RTROBOT_DIR}" CACHE PATH "Path to rtrobot directory" FORCE)
endif()

message(STATUS "Using RTROBOT_DIR=${RTROBOT_DIR}")

# Configure compilation flags 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${AUXFLAGS} -g -Wall -D_LINUX_ -std=c++14")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${AUXFLAGS}")

include(CTest)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -I${RTROBOT_DIR}/include -L${RTROBOT_DIR}/lib")

if(${DEBUG_ASAN})
  message("Forcing DEBUG compilation for ASAN")
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Set to debug" FORCE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
else()
  message("Configuring WITHOUT ASAN")
endif()

# Find Eigen3 for kinematics calculations
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(${${CMAKE_PROJECT_NAME}_SOURCE_DIR}/include)

if(HARDWARE_TARGET STREQUAL "robot")
  add_subdirectory(src/hardware/robothw)
endif()

if(HARDWARE_TARGET STREQUAL "simulation")
  add_subdirectory(src/hardware/mujocohw)
endif()

if(HARDWARE_TARGET STREQUAL "go1")
  add_subdirectory(src/hardware/go1hw)
endif()

add_subdirectory(src/quadruped)

add_subdirectory(src/supervisor)

