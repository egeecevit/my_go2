set(CONTROLSRC CoreModules.cc QuadrupedConfigs.cc MdlDrawSquare.cc MdlTrot.cc MdlStand.cc MdlSit.cc MdlLegControl.cc QuadrupedKinematics.cc MdlOrientationEstimator.cc MdlPosVelEstimator.cc MdlHWTest.cc)

add_library(quadruped STATIC ${CONTROLSRC})
install(TARGETS quadruped)

add_subdirectory(tests)
