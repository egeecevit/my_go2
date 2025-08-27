cmake_minimum_required(VERSION 3.5)
project(sampleproject)

option(DEBUG_ASAN "Enable ASAN debug option" OFF)
option(COMPILE_ROBOT "Enable compilation of robot code" ON)
option(COMPILE_SIMULATION "Enable compilation of simulation code" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if ( "$ENV{RTROBOT_DIR}" STREQUAL "")
  message( FATAL_ERROR "Please set RTROBOT_DIR environment variable.")
else ()
  set(RTROBOT_DIR "$ENV{RTROBOT_DIR}")
endif()

if ( "$ENV{QUADCONTROL_DIR}" STREQUAL "")
  message( FATAL_ERROR "Please set QUADCONTROL_DIR environment variable.")
else ()
  set(QUADCONTROL_DIR "$ENV{QUADCONTROL_DIR}")
endif()

# Configure compilation flags 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${AUXFLAGS} -g -Wall -D_LINUX_ -std=c++14")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${AUXFLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I${RTROBOT_DIR}/include -L${RTROBOT_DIR}/lib -I${QUADCONTROL_DIR}/include -L${QUADCONTROL_DIR}/lib" )
message(${RTROBOT_DIR})
message(${CMAKE_CXX_FLAGS})

# Find Eigen3 for kinematics calculations
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

set(SOURCES main.cc Supervisor.cc)

if (${COMPILE_ROBOT})
  set(ROBOTEXE robot)
  add_executable(${ROBOTEXE} ${SOURCES})
  target_link_libraries(${ROBOTEXE} robothw quadruped rtcore rtclient)
endif()

if(${COMPILE_SIMULATION})
  set(SIMEXE simulation)
  add_executable(${SIMEXE} ${SOURCES})
  message("Enabling MUJOCO simulation support")
  set(CMAKE_POLICY_DEFAULT_CMP0069 NEW) # INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
  set(CMAKE_POLICY_DEFAULT_CMP0072 NEW) # Default to GLVND if available.

  enable_language(C)
  enable_language(CXX)
  set(SAMPLE_STANDALONE ON)
  list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
  include(SampleOptions)
  include(SampleDependencies)

  # no-unused is to prevent Eigen's compilation errors due to the no-unused warning combined with AVX
  set(MUJOCO_SAMPLE_COMPILE_OPTIONS "${AVX_COMPILE_OPTIONS}" "${EXTRA_COMPILE_OPTIONS}" -Wno-unused)
  set(MUJOCO_SAMPLE_LINK_OPTIONS "${EXTRA_LINK_OPTIONS}")
  
  target_compile_options(${SIMEXE} PUBLIC ${MUJOCO_SAMPLE_COMPILE_OPTIONS}) 
  target_link_options(${SIMEXE} PRIVATE ${MUJOCO_SAMPLE_LINK_OPTIONS})
  target_link_libraries(${SIMEXE} mujocohw quadruped rtcore rtclient mujoco::mujoco glfw Threads::Threads)
endif()
