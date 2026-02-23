set(CONTROLSRC CoreModules.cc MdlDrawSquare.cc MdlLegControl.cc QuadrupedKinematics.cc MdlSineTest.cc)

add_library(quadruped STATIC ${CONTROLSRC})
install(TARGETS quadruped)

add_subdirectory(tests)
