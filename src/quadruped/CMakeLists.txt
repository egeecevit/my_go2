set(CONTROLSRC CoreModules.cc MdlDrawSquare.cc MdlLegControl.cc QuadrupedKinematics.cc)

add_library(control STATIC ${CONTROLSRC})

add_subdirectory(tests)