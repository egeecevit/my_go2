#ifndef MDLSIT_HH
#define MDLSIT_HH

#include "rtcore/Module.hh"
#include <Eigen/Dense>

#define SITMODULE_NAME "MdlSit"

class MdlLegControl;
class QuadrupedKinematics;

class MdlSit : public rtcore::Module {
public:
  enum Status { IDLE, ACTIVE, SETTLED, ERROR };

  MdlSit();
  ~MdlSit();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  Status getStatus() const { return _status; }

private:
  MdlLegControl *_legs[4] = {};
  QuadrupedKinematics *_kinematics = nullptr;
  Status _status = IDLE;

  // Configured sit target: joint angles from sit.toml, converted to foot
  // positions via FK at init time.
  Eigen::Vector3d _sitTarget[4];
  bool _sitTargetValid = false;

  // Activation-time foot positions (where we start the blend from)
  Eigen::Vector3d _footStart[4];

  double _duration = 2.0;
  double _startTime = 0.0;
  double _trackingErrorLimit = 0.5;

  bool _checkTrackingError() const;
};

#endif
