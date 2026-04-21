set(CONTROLSRC CoreModules.cc QuadrupedConfigs.cc MdlDrawSquare.cc MdlStand.cc MdlSit.cc MdlLegControl.cc QuadrupedKinematics.cc MdlSineTest.cc MdlHWTest.cc)

add_library(quadruped STATIC ${CONTROLSRC})
install(TARGETS quadruped)

add_subdirectory(tests)
