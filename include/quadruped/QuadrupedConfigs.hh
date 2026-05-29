#ifndef QUADRUPEDCONFIGS_HH
#define QUADRUPEDCONFIGS_HH

#include "quadruped/QuadrupedKinematics.hh"

// Per-hardware-target kinematic parameters. Used by MdlLegControl, MdlDrawSquare,
// MdlSit, and anything else that needs FK/IK with the right dimensions.
QuadrupedKinematics::params_t createGo1Config();
QuadrupedKinematics::params_t createGo2Config();

#endif
