set(CONTROLSRC CoreModules.cc QuadrupedConfigs.cc QuadrupedLegDynamics.cc MdlDrawSquare.cc MdlTrot.cc MdlStand.cc MdlSit.cc MdlLegControl.cc MdlConvexMPC.cc QuadrupedKinematics.cc MdlOrientationEstimator.cc MdlPosVelEstimator.cc MdlHWTest.cc)

# ---- qpOASES
#
# The QP solver named by the MIT Cheetah 3 convex MPC paper and used by the
# public Cheetah controller. Fetched at configure time and pinned to the exact
# upstream release so a fresh configure is reproducible. There is deliberately
# no system-package branch: a distribution qpOASES would be a different version
# with different active-set behavior, and silently solving the MPC with it is
# worse than failing to configure. A download failure therefore stops the build.
include(FetchContent)

set(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

# qpOASES 3.2.2 still declares cmake_minimum_required(VERSION 2.6), which CMake
# 4 rejects outright. This is the documented compatibility escape hatch for an
# unmaintained dependency; it is a normal variable, so it reaches the fetched
# subdirectory and nothing above this file.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# qpOASES carries its own install rules -- thirty headers, the library, and a
# CMake package config -- and this project's install prefix is the repository
# root, so left alone `cmake --install` scatters all of it through the source
# tree. Nothing here consumes the installed copy: we compile against the fetched
# sources and link the target directly. So the three install destinations are
# pointed at a corner of the build directory for the duration of the fetch and
# restored afterwards, which leaves the repository clean without patching an
# upstream release.
#
# The unsets matter as much as the sets: install(TARGETS quadruped) below reads
# CMAKE_INSTALL_LIBDIR too, and would follow qpOASES into the build tree.
set(_qpoases_stage ${CMAKE_CURRENT_BINARY_DIR}/qpoases-stage)
set(CMAKE_INSTALL_INCLUDEDIR ${_qpoases_stage}/include)
set(CMAKE_INSTALL_LIBDIR ${_qpoases_stage}/lib)
set(CMAKE_INSTALL_BINDIR ${_qpoases_stage}/bin)

# MuJoCo's dependency setup turns BUILD_SHARED_LIBS on, and qpOASES honours it,
# which would leave every installed binary with a runtime dependency on a .so
# living inside the build tree. Static instead: it is a small solver, it is the
# only consumer, and bin/ should not stop working because build/ was cleaned.
set(BUILD_SHARED_LIBS OFF)

FetchContent_Declare(
  qpoases
  GIT_REPOSITORY https://github.com/coin-or/qpOASES.git
  GIT_TAG releases/3.2.2
)
FetchContent_MakeAvailable(qpoases)

unset(BUILD_SHARED_LIBS)
unset(CMAKE_INSTALL_INCLUDEDIR)
unset(CMAKE_INSTALL_LIBDIR)
unset(CMAKE_INSTALL_BINDIR)

add_library(quadruped STATIC ${CONTROLSRC})

# PUBLIC on both counts: the tests link `quadruped` and pull in MdlConvexMPC.o,
# which needs the solver's symbols. qpOASES 3.2.2 predates usage requirements
# and exports no include directory of its own, so its header path is added here.
target_link_libraries(quadruped PUBLIC qpOASES)
target_include_directories(quadruped PUBLIC ${qpoases_SOURCE_DIR}/include)

install(TARGETS quadruped)

add_subdirectory(tests)
