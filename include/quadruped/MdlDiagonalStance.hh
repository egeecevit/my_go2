/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

#ifndef MDLDIAGONALSTANCE_HH
#define MDLDIAGONALSTANCE_HH

#include "rtcore/Module.hh"
#include <Eigen/Dense>

#define DIAGSTANCEMODULE_NAME "MdlDiagonalStance"

class MdlLegControl;

/** \brief Four-footed stand -> FL and RR on the body centreline, and back.

  Steps the diagonal feet one at a time with CoM shifts in between so no foot
  slides under load; SETTLED once FL/RR sit at (+-stance_x, 0, -0.278) and the
  body is centred, which is the posture MdlAMB takes over from.

  setReverse(true) is the way back from MdlAMB's settled four-foot support:
  every foot steps to the spot it had under the body when the first way in
  started, so the robot ends in the stand's footprint again. */
class MdlDiagonalStance : public rtcore::Module {
public:
  enum Status { IDLE, ACTIVE, SETTLED, ERROR };

  MdlDiagonalStance();
  ~MdlDiagonalStance();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  Status getStatus() const { return _status; }
  /** \brief Direction of the next activation; the way back (true) needs a
      way in before it, which records the footprint to return to. */
  void setReverse(bool reverse) { _reverse = reverse; }

private:
  MdlLegControl *_legs[4] = {};
  Status _status = IDLE;

  // [diagstance] keys; defaults are the amb.toml values.
  Eigen::Vector2d _shift1 = Eigen::Vector2d(0.10, 0.05);
  Eigen::Vector2d _shift2 = Eigen::Vector2d(-0.02, 0.01);
  double _shiftDuration = 1.0;
  double _stepDuration = 0.8;
  double _stepClearance = 0.05;
  double _trackingErrorLimit = 0.5;
  // Joint PD for the foot targets. The legcontrol default (100 / 5) sags the
  // loaded legs 2-3 cm, which tilts the body and leaves FL/RR 2-4 cm off their
  // hand-off spots; 400 / 10 brings that under 7 mm (measured in sim).
  double _kp = 400.0;
  double _kd = 10.0;

  // [amb] keys that fix the hand-off posture MdlAMB expects.
  double _stanceX = 0.15;
  double _baseZ = 0.30;

  // RR steps first: with the stand's feet 6.5 cm ahead of the body origin, FL at
  // (stance_x, 0) would leave the FL/FR/RL triangle for the RR step a sliver
  // (inradius 2.3 cm); RR first keeps both single-foot stages above 5 cm.
  enum Stage { SHIFT1, STEP_RR, SHIFT2, STEP_FL, CENTRE };
  int _stage = SHIFT1;         // the way back counts shift/step pairs instead
  bool _last = false;          // the stage that ends in SETTLED
  double _stageStart = 0.0;
  double _duration = 1.0;
  int _stepLeg = -1;           // leg carrying the clearance hump, -1 in shift stages

  // The way back. MdlAMB's swing feet land wherever the balance left them, so
  // its shifts are planned from the feet as found, not read from a table.
  bool _reverse = false;
  bool _haveStand = false;
  Eigen::Vector3d _stand[4];   // level-frame footprint at the first way in
  bool _placed[4] = {};        // foot already back on its _stand spot
  int _nextLeg = -1;           // foot the current shift makes room for
  Eigen::Vector3d _shift = Eigen::Vector3d::Zero();
  // Body -> gravity-aligned frame at activation. The stand hands over a body
  // pitched several degrees, so the stage geometry lives in the level frame and
  // SHIFT1 blends this tilt out; from then on body and level frames coincide.
  Eigen::AngleAxisd _tilt = Eigen::AngleAxisd::Identity();
  // Ground-anchored foot positions in the unshifted level frame: a planted foot
  // sits at _p0 - delta while the body is shifted by delta, and a stepped foot's
  // entry is overwritten with its nominal spot so CENTRE (delta -> 0) lands it
  // there exactly.
  Eigen::Vector3d _p0[4];
  Eigen::Vector3d _pStart[4];  // commanded level-frame positions at the stage boundaries
  Eigen::Vector3d _pEnd[4];

  bool _checkTrackingError() const;
  void _planReturn(int stage);
  void _beginStage(int stage, double t);
};

#endif
