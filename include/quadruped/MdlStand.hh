#ifndef MDLSTAND_HH
#define MDLSTAND_HH

#include "rtcore/Module.hh"
#include <Eigen/Dense>

#define STANDMODULE_NAME "MdlStand"

class MdlLegControl;

class MdlStand : public rtcore::Module {
public:
  enum Status { IDLE, ACTIVE, SETTLED, ERROR };

  MdlStand();
  ~MdlStand();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  void setTargetHeight(double delta_h, double duration);
  void setTargetHeight(double delta_h);  // uses config duration
  Status getStatus() const { return _status; }

private:
  MdlLegControl *_legs[4] = {};
  Status _status = IDLE;

  Eigen::Vector3d _footB0[4];
  bool _footCaptured = false;

  double _deltaHStart = 0.0;  // height offset at start of current transition
  double _deltaHEnd = 0.0;    // target height offset
  double _configDuration = 2.0;
  double _duration = 2.0;
  double _startTime = 0.0;

  double _trackingErrorLimit = 0.5;

  static double _quintic(double tau);
  static double _quinticDot(double tau);
  bool _checkTrackingError() const;
};

#endif
