set(CONTROLSRC CoreModules.cc MdlDrawSquare.cc MdlStand.cc MdlLegControl.cc QuadrupedKinematics.cc MdlSineTest.cc MdlHWTest.cc)

add_library(quadruped STATIC ${CONTROLSRC})
install(TARGETS quadruped)

add_subdirectory(tests)
