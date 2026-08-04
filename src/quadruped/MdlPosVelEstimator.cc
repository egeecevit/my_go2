/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#include "quadruped/MdlPosVelEstimator.hh"

#include <algorithm>
#include <cmath>

#include "hardware/MotorHW.hh"
#include "quadruped/MdlOrientationEstimator.hh"
#include "quadruped/MdlTrot.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedKinematics.hh"
#include "rtcore/ConfigTable.hh"
#include "rtcore/LogServer.hh"
#include "rtcore/ModuleManager.hh"

#define DBGPRINT(...)  // printf(__VA_ARGS__)

using namespace rtcore;

// Simulation introspection and debug drawing, defined in MdlSimDriver.cc. These
// only exist in the simulation target; on go1 and robot they resolve to null and
// every use below is guarded. Same convention as simPollKey().
extern bool simGetGroundTruth(double pos[3], double quat[4], double linvel[3],
                              double angvel[3]) __attribute__((weak));
extern bool simGetFootContacts(bool contacts[4], double forces[4]) __attribute__((weak));
extern bool simGetFootPositions(double pos[4][3]) __attribute__((weak));
extern void simDebugClear() __attribute__((weak));
extern void simDebugSphere(const double pos[3], double radius,
                           const double rgba[4]) __attribute__((weak));
extern void simDebugLine(const double from[3], const double to[3], double width,
                         const double rgba[4]) __attribute__((weak));

MdlPosVelEstimator::MdlPosVelEstimator()
    : Module(POSVELMODULE_NAME, 0, MULTI_USER) {
  DBGPRINT("MdlPosVelEstimator::MdlPosVelEstimator\n");
  for (int i = 0; i < NUM_LEGS; i++) {
    _jointAngles[i].setZero();
    _jointVel[i].setZero();
    _jointTorques[i].setZero();
    _footPosBody[i].setZero();
    _footVelBody[i].setZero();
    _footJacobian[i].setIdentity();
    _footTruePos[i].setZero();
    _stanceStartPos[i].setZero();
  }
}

MdlPosVelEstimator::~MdlPosVelEstimator() {
  DBGPRINT("MdlPosVelEstimator::~MdlPosVelEstimator\n");
}

// ===========================================================================
//  The filter
// ===========================================================================

double MdlPosVelEstimator::trustFromPhase(double phase, double window) {
  if (phase > 1.0) phase = 1.0;
  if (phase < 0.0) phase = 0.0;
  // A zero window means no ramp at all, so anything in stance is fully
  // trusted. Guarding here rather than letting the divisions run: the MIT
  // sources would produce 0/0 and poison the whole covariance.
  if (!(window > 0.0)) return 1.0;

  if (phase < window) return phase / window;
  if (phase > 1.0 - window) return (1.0 - phase) / window;
  return 1.0;
}

void MdlPosVelEstimator::setParams(const params_t& params) {
  _params = params;
  if (!(_params.dt > 0.0)) _params.dt = 0.001;
  if (!(_params.trust_window >= 0.0)) _params.trust_window = 0.0;
  if (_params.trust_window > 0.5) _params.trust_window = 0.5;
}

void MdlPosVelEstimator::reset() {
  const double dt = _params.dt;

  _xhat.setZero();

  // Double integrator on the body, and foot positions that are constant under
  // the prediction model: a foot on the ground does not move, and one in the
  // air is handled by inflating its process noise rather than by modelling it.
  _A.setZero();
  _A.block<3, 3>(0, 0).setIdentity();
  _A.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
  _A.block<3, 3>(3, 3).setIdentity();
  _A.block<12, 12>(6, 6).setIdentity();

  _B.setZero();
  _B.block<3, 3>(3, 0) = dt * Eigen::Matrix3d::Identity();

  // C1 selects the body position out of (r, v); C2 selects the body velocity.
  Eigen::Matrix<double, 3, 6> C1, C2;
  C1 << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero();
  C2 << Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();

  _C.setZero();
  for (int i = 0; i < NUM_LEGS; i++) {
    _C.block<3, 6>(3 * i, 0) = C1;       // rows  0-11: r - p_i
    _C.block<3, 6>(12 + 3 * i, 0) = C2;  // rows 12-23: v
    _C(24 + i, 6 + 3 * i + 2) = 1.0;     // rows 24-27: world z of foot i
  }
  _C.block<12, 12>(0, 6) = -Eigen::Matrix<double, 12, 12>::Identity();

  // Large and uncorrelated: the filter is told it knows nothing yet, and the
  // measurements bring it in. The horizontal position block never really
  // shrinks, which is honest, since nothing here observes absolute x and y.
  _P.setIdentity();
  _P *= 100.0;

  // Fixed covariance shapes. The configured noise values are dimensionless
  // multipliers on these rather than physical densities, which is why they can
  // all sit within an order of magnitude of each other.
  _Q0.setIdentity();
  _Q0.block<3, 3>(0, 0) = (dt / 20.0) * Eigen::Matrix3d::Identity();
  // The 9.8 here is not gravity, despite appearances. It is the fixed ratio
  // between the position and velocity process noise shapes, chosen so that both
  // can be driven from configuration values of the same magnitude. Gravity
  // enters the filter once, in step(), and has nothing to do with this line.
  _Q0.block<3, 3>(3, 3) = (dt * 9.8 / 20.0) * Eigen::Matrix3d::Identity();
  _Q0.block<12, 12>(6, 6) = dt * Eigen::Matrix<double, 12, 12>::Identity();

  _R0.setIdentity();

  _Rbw.setIdentity();
  _vBody.setZero();
  for (int i = 0; i < NUM_LEGS; i++) _trust[i] = 0.0;
  _needSeed = true;
  _ready = false;
}

void MdlPosVelEstimator::_seedFromKinematics(const Eigen::Matrix3d& Rbw,
                                             const Eigen::Vector3d footPos[NUM_LEGS],
                                             const double phase[NUM_LEGS]) {
  // Wait for a foot worth measuring from. The force detector needs a few cycles
  // to latch and a gait may start mid swing, so the first cycle often has
  // nothing to say; consuming the seed then would waste it.
  double sum = 0.0;
  int n = 0;
  for (int i = 0; i < NUM_LEGS; i++) {
    if (phase[i] <= 0.0) continue;
    sum += (Rbw * footPos[i]).z();
    n++;
  }
  if (n == 0) return;

  // Place the body so the contacting feet rest on the ground datum. Horizontal
  // position has no reference at all in this filter, so it starts at zero, and
  // every position error reported afterwards is relative to wherever that was.
  _xhat.setZero();
  _xhat(2) = _params.foot_ground_height - sum / (double)n;
  for (int i = 0; i < NUM_LEGS; i++)
    _xhat.segment<3>(6 + 3 * i) = _xhat.head<3>() + Rbw * footPos[i];

  // The covariance is deliberately left alone. Seeding the mean removes the
  // startup transient if the seed is good; leaving P large means the filter
  // discards the seed within a few cycles if it is not. Shrinking P as well
  // would lock in a bad guess, which is the failure worth avoiding.
  _needSeed = false;
}

void MdlPosVelEstimator::step(const Eigen::Matrix3d& Rbw, const Eigen::Vector3d& aWorld,
                              const Eigen::Vector3d& omegaBody,
                              const Eigen::Vector3d footPos[NUM_LEGS],
                              const Eigen::Vector3d footVel[NUM_LEGS],
                              const double phase[NUM_LEGS]) {
  _Rbw = Rbw;

  Eigen::Matrix<double, DIM, DIM> Q = Eigen::Matrix<double, DIM, DIM>::Identity();
  Q.block<3, 3>(0, 0) = _Q0.block<3, 3>(0, 0) * _params.imu_process_noise_position;
  Q.block<3, 3>(3, 3) = _Q0.block<3, 3>(3, 3) * _params.imu_process_noise_velocity;
  Q.block<12, 12>(6, 6) = _Q0.block<12, 12>(6, 6) * _params.foot_process_noise_position;

  Eigen::Matrix<double, MEAS, MEAS> R = Eigen::Matrix<double, MEAS, MEAS>::Identity();
  R.block<12, 12>(0, 0) = _R0.block<12, 12>(0, 0) * _params.foot_sensor_noise_position;
  R.block<12, 12>(12, 12) = _R0.block<12, 12>(12, 12) * _params.foot_sensor_noise_velocity;
  R.block<4, 4>(24, 24) = _R0.block<4, 4>(24, 24) * _params.foot_height_sensor_noise;

  // The accelerometer reports specific force, so at rest it reads +9.81 upward
  // and adding gravity gives zero. Rbw has already been applied upstream to put
  // it in the world frame.
  const Eigen::Vector3d a = aWorld + _params.gravity;

  if (_needSeed) {
    if (_params.warm_start)
      _seedFromKinematics(Rbw, footPos, phase);
    else
      _needSeed = false;
  }

  // Taken before the propagation below, so the blends further down mix the
  // measurement against the same estimate the residual is computed from.
  const Eigen::Vector3d r0 = _xhat.head<3>();
  const Eigen::Vector3d v0 = _xhat.segment<3>(3);

  Eigen::Matrix<double, 12, 1> ps, vs;
  Eigen::Vector4d pzs = Eigen::Vector4d::Zero();

  for (int i = 0; i < NUM_LEGS; i++) {
    const int i1 = 3 * i;

    // Foot position relative to the body. The MIT sources add a hip offset here
    // because their leg kinematics are expressed in the hip frame; ours already
    // return a body frame position, so there is nothing to add.
    const Eigen::Vector3d& p_rel = footPos[i];
    const Eigen::Vector3d& dp_rel = footVel[i];

    // Into the world frame. The velocity picks up the transport term: a foot
    // held still relative to the body still moves through the world when the
    // body rotates, and omitting it makes the filter read a turn as translation.
    const Eigen::Vector3d p_f = Rbw * p_rel;
    const Eigen::Vector3d dp_f = Rbw * (omegaBody.cross(p_rel) + dp_rel);

    const int qindex = 6 + i1, rindex2 = 12 + i1, rindex3 = 24 + i;

    const double trust = trustFromPhase(phase[i], _params.trust_window);
    _trust[i] = trust;

    // An untrusted foot is not excluded, it is doubted. Its own process noise
    // grows so the filter lets it wander, and the measurements that assert it
    // is standing still are de-weighted to match. Note that the position rows
    // are deliberately left alone: an airborne foot's position measurement is
    // exactly what re-anchors it when it lands.
    const double suspect = 1.0 + (1.0 - trust) * _params.high_suspect_number;
    Q.block<3, 3>(qindex, qindex) *= suspect;
    R.block<3, 3>(rindex2, rindex2) *= suspect;
    R(rindex3, rindex3) *= suspect;

    // Body-to-foot vector, negated to match the sign of the C rows above.
    ps.segment<3>(i1) = -p_f;

    // Body velocity from foot velocity, which holds only if the foot is
    // stationary. Where it is not trusted the measurement fades into the
    // current estimate, making the residual vanish rather than pull.
    vs.segment<3>(i1) = (1.0 - trust) * v0 + trust * (-dp_f);

    // Foot height against flat ground. Same fade, and the datum is the foot
    // sphere radius rather than the MIT sources' zero, because leg kinematics
    // report the centre of the foot and not its contact point.
    pzs(i) = (1.0 - trust) * (r0.z() + p_f.z()) + trust * _params.foot_ground_height;
  }

  Eigen::Matrix<double, MEAS, 1> y;
  y << ps, vs, pzs;

  _xhat = _A * _xhat + _B * a;
  const Eigen::Matrix<double, DIM, DIM> Pm = _A * _P * _A.transpose() + Q;
  const Eigen::Matrix<double, DIM, MEAS> Ct = _C.transpose();
  const Eigen::Matrix<double, MEAS, 1> ey = y - _C * _xhat;
  const Eigen::Matrix<double, MEAS, MEAS> S = _C * Pm * Ct + R;

  // One factorization, reused for both solves. The MIT sources factor the same
  // matrix twice and note it as a todo.
  const Eigen::PartialPivLU<Eigen::Matrix<double, MEAS, MEAS> > lu(S);
  _xhat += Pm * Ct * lu.solve(ey);
  _P = (Eigen::Matrix<double, DIM, DIM>::Identity() - Pm * Ct * lu.solve(_C)) * Pm;

  // eval() is required: the right hand side reads _P while assigning to it.
  _P = 0.5 * (_P + _P.transpose()).eval();

  // Horizontal position is unobservable, so its variance grows without bound
  // and drags its correlations with everything else along. Left alone it
  // eventually swamps the numerics of the whole matrix. Cutting it back keeps
  // the filter well conditioned over long runs, at the cost of the first two
  // covariance diagonals no longer meaning anything: they are held near the
  // clamp, not tracking a real uncertainty.
  if (_P.block<2, 2>(0, 0).determinant() > 1e-6) {
    _P.block<2, 16>(0, 2).setZero();
    _P.block<16, 2>(2, 0).setZero();
    _P.block<2, 2>(0, 0) /= 10.0;
  }

  _vBody = Rbw.transpose() * _xhat.segment<3>(3);
  _ready = true;
}

// ===========================================================================
//  Module interface
// ===========================================================================

void MdlPosVelEstimator::init() {
  DBGPRINT("MdlPosVelEstimator::init\n");

  // Own kinematics rather than borrowing the MdlLegControl modules: those are
  // SINGLE_USER and owned by whichever behavior is running, and the estimator
  // must never contend with a controller for them.
  ConfigTable hwConfig;
  std::string hwlib;
  if (_mgr->getConfigTable("hardware", hwConfig))
    hwlib = hwConfig.getString("library", "");

  if (hwlib == "go1hw")
    _kinematics = new QuadrupedKinematics(createGo1Config());
  else
    _kinematics = new QuadrupedKinematics(createGo2Config());

  _dt = CLOCK_TO_SEC(_mgr->getStepPeriod());
  _readConfig();
  reset();

  // The attitude stage runs one order earlier in the same cycle, and this
  // module is useless without it. AddCoreModules creates it first, so the
  // lookup succeeds; failing loudly here means a future reordering shows up as
  // a startup error rather than as a silently wrong estimate.
  _orientation = (MdlOrientationEstimator*)_mgr->findModule(ORIENTATIONMODULE_NAME, 0);
  if (!_orientation)
    _mgr->fatalError(POSVELMODULE_NAME,
                     "%s must be created before %s in AddCoreModules()",
                     ORIENTATIONMODULE_NAME, POSVELMODULE_NAME);

  _logserver = (LogServer*)_mgr->findModule(LOGSERVER_NAME, 0);
  if (_logserver) {
    _logserver->registerVar(LOG_DOUBLE, 9, POSVELMODULE_NAME, "state",
                            (unsigned char*)_logState);
    _logserver->registerVar(LOG_DOUBLE, 12, POSVELMODULE_NAME, "footholds",
                            (unsigned char*)_logFootholds);
    _logserver->registerVar(LOG_DOUBLE, 12, POSVELMODULE_NAME, "error",
                            (unsigned char*)_logError);
    _logserver->registerVar(LOG_DOUBLE, DIM, POSVELMODULE_NAME, "cov",
                            (unsigned char*)_logCov);
    _logserver->registerVar(LOG_DOUBLE, 8, POSVELMODULE_NAME, "contacts",
                            (unsigned char*)_logContacts);
    _logserver->registerVar(LOG_DOUBLE, 8, POSVELMODULE_NAME, "slip",
                            (unsigned char*)_logSlip);
  }
}

void MdlPosVelEstimator::uninit() {
  DBGPRINT("MdlPosVelEstimator::uninit\n");

  if (_logserver) {
    _logserver->deleteVar(POSVELMODULE_NAME, "state");
    _logserver->deleteVar(POSVELMODULE_NAME, "footholds");
    _logserver->deleteVar(POSVELMODULE_NAME, "error");
    _logserver->deleteVar(POSVELMODULE_NAME, "cov");
    _logserver->deleteVar(POSVELMODULE_NAME, "contacts");
    _logserver->deleteVar(POSVELMODULE_NAME, "slip");
    _logserver = nullptr;
  }

  if (_kinematics) delete _kinematics;
  _kinematics = nullptr;
}

void MdlPosVelEstimator::_readConfig() {
  ConfigTable config;
  if (!_mgr->getConfigTable("posvelestimator", config)) {
    _mgr->warning(POSVELMODULE_NAME,
                  "No [posvelestimator] config table; using built-in defaults");
    return;
  }

  _enable = config.getBool("enable", true);

  params_t p;
  p.dt = _dt;
  p.imu_process_noise_position =
      config.getDouble("imu_process_noise_position", p.imu_process_noise_position);
  p.imu_process_noise_velocity =
      config.getDouble("imu_process_noise_velocity", p.imu_process_noise_velocity);
  p.foot_process_noise_position =
      config.getDouble("foot_process_noise_position", p.foot_process_noise_position);
  p.foot_sensor_noise_position =
      config.getDouble("foot_sensor_noise_position", p.foot_sensor_noise_position);
  p.foot_sensor_noise_velocity =
      config.getDouble("foot_sensor_noise_velocity", p.foot_sensor_noise_velocity);
  p.foot_height_sensor_noise =
      config.getDouble("foot_height_sensor_noise", p.foot_height_sensor_noise);
  p.trust_window = config.getDouble("trust_window", p.trust_window);
  p.high_suspect_number = config.getDouble("high_suspect_number", p.high_suspect_number);
  p.foot_ground_height = config.getDouble("foot_ground_height", p.foot_ground_height);
  p.warm_start = config.getBool("warm_start", p.warm_start);
  p.use_ground_truth = config.getBool("use_ground_truth", p.use_ground_truth);

  ConfigArray gravity;
  if (config.getArray("gravity", gravity) && gravity.size() >= 3)
    p.gravity = Eigen::Vector3d(gravity.getDoubleAt(0, 0.0), gravity.getDoubleAt(1, 0.0),
                                gravity.getDoubleAt(2, -9.81));

  if (p.use_ground_truth && !simGetGroundTruth) {
    _mgr->warning(POSVELMODULE_NAME,
                  "use_ground_truth needs the simulation target; running the filter");
    p.use_ground_truth = false;
  }

  setParams(p);

  // Keys from the Bloesch filter this replaces. Silently ignoring a stale
  // override left in a version or robot directory would mean running with
  // settings nobody intended.
  if (config.getDouble("foot_radius", -1.0) >= 0.0)
    _mgr->warning(POSVELMODULE_NAME,
                  "foot_radius is obsolete; it is now foot_ground_height and is an "
                  "active height measurement rather than a fixed datum");
  if (config.getDouble("swing_foot_noise", -1.0) >= 0.0)
    _mgr->warning(POSVELMODULE_NAME,
                  "swing_foot_noise is obsolete; swing feet are now handled by the "
                  "contact trust ramp (trust_window, high_suspect_number)");

  _phaseSource = config.getString("contact_phase_source", _phaseSource);
  if (_phaseSource != "auto" && _phaseSource != "gait" && _phaseSource != "force") {
    _mgr->warning(POSVELMODULE_NAME, "Unknown contact_phase_source '%s'; using 'auto'",
                  _phaseSource.c_str());
    _phaseSource = "auto";
  }

  _contactSource = config.getString("contact_source", _contactSource);
  if (_contactSource != "torque" && _contactSource != "simtruth") {
    _mgr->warning(POSVELMODULE_NAME, "Unknown contact_source '%s'; using 'torque'",
                  _contactSource.c_str());
    _contactSource = "torque";
  }
  if (_contactSource == "simtruth" && !simGetFootContacts) {
    _mgr->warning(POSVELMODULE_NAME,
                  "contact_source 'simtruth' needs the simulation target; "
                  "falling back to 'torque'");
    _contactSource = "torque";
  }
  _contactOnThreshold = config.getDouble("contact_on_threshold", _contactOnThreshold);
  _contactOffThreshold = config.getDouble("contact_off_threshold", _contactOffThreshold);
  _contactDebounce = (int)config.getInt("contact_debounce", _contactDebounce);
  _jacobianDamping = config.getDouble("jacobian_damping", _jacobianDamping);
  _contactForceSign = config.getDouble("contact_force_sign", _contactForceSign);

  ConfigTable viz;
  if (config.getTable("viz", viz)) {
    _vizEnable = viz.getBool("enable", true);
    _vizTrail = viz.getBool("show_trail", true);
    _vizTrailLength = (int)viz.getInt("trail_length", 200);
    if (_vizTrailLength > MAX_TRAIL) _vizTrailLength = MAX_TRAIL;
    if (_vizTrailLength < 0) _vizTrailLength = 0;
    _vizTrailDecimation = (int)viz.getInt("trail_decimation", 20);
    if (_vizTrailDecimation < 1) _vizTrailDecimation = 1;
    _vizMarkerSize = viz.getDouble("marker_size", 0.03);
  }

  ConfigTable truth;
  if (config.getTable("groundtruth", truth)) {
    _truthEnable = truth.getBool("enable", true);
    _truthReportPeriod = truth.getDouble("report_period", 2.0);
    _pathSamplePeriod = truth.getDouble("path_sample_period", _pathSamplePeriod);
    if (_pathSamplePeriod < _dt) _pathSamplePeriod = _dt;
    _pathFilterTau = truth.getDouble("path_filter_tau", _pathFilterTau);
    if (_pathFilterTau < 0.0) _pathFilterTau = 0.0;
  }
}

void MdlPosVelEstimator::activate() {
  DBGPRINT("MdlPosVelEstimator::activate\n");

  reset();

  // MdlTrot is created after this module, so init() is too early to find it.
  // Read only: it is SINGLE_USER and the Supervisor owns it, so grabbing it
  // would take a behavior's legs out from under it.
  if (!_trotSearched) {
    _trot = (MdlTrot*)_mgr->findModule(TROTMODULE_NAME, 0);
    _trotSearched = true;
  }

  _startTime = _mgr->readTime();
  _lastReportTime = _startTime;

  // Reset the error statistics so each activation is measured on its own.
  _sumSqPos.setZero();
  _sumSqVel.setZero();
  _sumSqAtt.setZero();
  _errorSamples = 0;
  _distanceTravelled = 0.0;
  _havePathSample = false;
  _haveFootTruth = false;
  _contactAgree = 0;
  _contactSamples = 0;
  for (int i = 0; i < NUM_LEGS; i++) {
    _slipSum[i] = 0.0;
    _slipMax[i] = 0.0;
    _stanceCount[i] = 0;
    _slipTracking[i] = false;
    _contacts[i] = false;
    _contactCount[i] = 0;
    _contactPhase[i] = 0.0;
    _footForce[i] = 0.0;
    _trustSum[i] = 0.0;
  }
  _trustSamples = 0;
  _slipSumSqAxis.setZero();
  _stanceTimeTotal = 0.0;
  _trailCount = 0;
  _trailHead = 0;
  _trailTick = 0;
  _comparison = comparison_t();
}

void MdlPosVelEstimator::deactivate() {
  DBGPRINT("MdlPosVelEstimator::deactivate\n");
  if (simDebugClear && _vizEnable) simDebugClear();
}

void MdlPosVelEstimator::update() {
  if (!_enable) return;
  if (!_orientation || !_orientation->isReady()) return;

  _readSensors();
  _detectContacts();
  _readContactPhase();

  const Eigen::Matrix3d& Rbw = _orientation->getRotation();

  if (_params.use_ground_truth && simGetGroundTruth) {
    // Straight substitution of the simulator's own answer, so that a run which
    // still looks wrong points at something other than this filter.
    double pos[3], quat[4], linvel[3], angvel[3];
    if (simGetGroundTruth(pos, quat, linvel, angvel)) {
      _xhat.head<3>() = Eigen::Vector3d(pos[0], pos[1], pos[2]);
      _xhat.segment<3>(3) = Eigen::Vector3d(linvel[0], linvel[1], linvel[2]);
      for (int i = 0; i < NUM_LEGS; i++)
        _xhat.segment<3>(6 + 3 * i) = _xhat.head<3>() + Rbw * _footPosBody[i];
      _Rbw = Rbw;
      _vBody = Rbw.transpose() * _xhat.segment<3>(3);
      for (int i = 0; i < NUM_LEGS; i++)
        _trust[i] = trustFromPhase(_contactPhase[i], _params.trust_window);
      _ready = true;
    }
  } else {
    step(Rbw, _orientation->getAccelerationWorld(), _orientation->getAngularVelocity(),
         _footPosBody, _footVelBody, _contactPhase);
  }

  for (int i = 0; i < NUM_LEGS; i++) _trustSum[i] += _trust[i];
  _trustSamples++;

  // Refreshes the foot truth cache that compareWithGroundTruth() reads, so it
  // has to come first.
  _updateSlipStats();

  // Evaluated once and shared by the statistics, the log buffers and the
  // overlay, all of which run every cycle.
  compareWithGroundTruth(_comparison);

  _updateGroundTruthStats();
  _refreshLogBuffers();
  _updateVisualization();
}

// ---------------------------------------------------------------------------
//  Sensing
// ---------------------------------------------------------------------------

void MdlPosVelEstimator::_readSensors() {
  MotorHW* motorhw = MotorHW::instance();
  if (!motorhw) return;

  for (int leg = 0; leg < NUM_LEGS; leg++) {
    for (int j = 0; j < 3; j++) {
      // Motor index convention shared with MdlLegControl: 3*leg + joint.
      const int idx = 3 * leg + j;
      MotorHW::state_t state;
      motorhw->getState(idx, state);
      _jointAngles[leg][j] = state.pos;
      _jointVel[leg][j] = state.vel;
      _jointTorques[leg][j] = state.tau;
    }

    // Unchecked FK: measured angles can legitimately sit slightly outside the
    // configured limits, and the estimator must not reject them for that.
    _kinematics->forwardKinematicsUnchecked(leg, _jointAngles[leg], _footPosBody[leg]);
    _kinematics->jacobian(leg, _jointAngles[leg], _footJacobian[leg]);

    // The measurement the whole filter turns on. A stationary foot means the
    // body is moving at exactly the negative of this, which is how a linear
    // filter gets a velocity without differentiating anything.
    _footVelBody[leg] = _footJacobian[leg] * _jointVel[leg];
  }
}

void MdlPosVelEstimator::_readContactPhase() {
  double gaitPhase[NUM_LEGS];

  if (_phaseSource != "force" && _trot && _trot->getState() == Module::ACTIVE &&
      _trot->getStancePhase(gaitPhase)) {
    // The gait is open loop, so its schedule is not an estimate of where the
    // feet are in stance, it is the definition.
    for (int i = 0; i < NUM_LEGS; i++) _contactPhase[i] = gaitPhase[i];
    return;
  }

  if (_phaseSource == "gait") {
    // No schedule to read, but that does not mean the feet are off the ground.
    // Standing, sitting and drawing all keep the robot planted.
    for (int i = 0; i < NUM_LEGS; i++) _contactPhase[i] = PLANTED_PHASE;
    return;
  }

  // A boolean carries no progress information, so it maps to the middle of
  // stance, where trust is one. Mapping it to 1.0 instead would put every
  // confidently detected foot at the far edge of the ramp, where trust is zero,
  // and the filter would ignore exactly the feet it should be using.
  for (int i = 0; i < NUM_LEGS; i++)
    _contactPhase[i] = _contacts[i] ? PLANTED_PHASE : 0.0;
}

void MdlPosVelEstimator::_latchContact(int leg, double normalForce) {
  _footForce[leg] = normalForce;

  // Schmitt trigger plus a debounce counter. A single threshold chatters badly
  // around touchdown and liftoff. Both contact sources come through here: the
  // simulator's contact force is exact but it still rings as the foot settles,
  // so it needs the same treatment the torque estimate does.
  const double threshold = _contacts[leg] ? _contactOffThreshold : _contactOnThreshold;
  const bool above = normalForce > threshold;

  if (above != _contacts[leg]) {
    if (++_contactCount[leg] >= _contactDebounce) {
      _contacts[leg] = above;
      _contactCount[leg] = 0;
    }
  } else {
    _contactCount[leg] = 0;
  }
}

void MdlPosVelEstimator::_detectContacts() {
  if (_contactSource == "simtruth" && simGetFootContacts) {
    double truthForce[NUM_LEGS];
    if (simGetFootContacts(nullptr, truthForce)) {
      for (int i = 0; i < NUM_LEGS; i++) _latchContact(i, truthForce[i]);
      return;
    }
  }

  // Torque based estimate. In static equilibrium the joint torques of one leg
  // relate to the force the ground applies at the foot by tau = -s J^T f, so
  // f = -s (J^T)^-1 tau, with the sign s absorbing the actuator convention of
  // the target. A robot standing on level ground must produce an upward force;
  // if contacts never latch, flip contact_force_sign.
  //
  // Project onto the true vertical using the estimated attitude, so the test
  // stays meaningful when the body is pitched or rolled.
  const Eigen::Vector3d up = _Rbw.transpose() * Eigen::Vector3d::UnitZ();

  for (int leg = 0; leg < NUM_LEGS; leg++) {
    const Eigen::Matrix3d Jt = _footJacobian[leg].transpose();

    // Damped least squares: near full leg extension the Jacobian becomes
    // ill-conditioned and a plain solve would produce enormous forces.
    const double lam2 = _jacobianDamping * _jacobianDamping;
    Eigen::Matrix3d JtJ = Jt.transpose() * Jt;
    JtJ.diagonal().array() += lam2;
    const Eigen::Vector3d force =
        -_contactForceSign * JtJ.inverse() * Jt.transpose() * _jointTorques[leg];

    _latchContact(leg, force.dot(up));
  }
}

// ---------------------------------------------------------------------------
//  Ground truth comparison
// ---------------------------------------------------------------------------

bool MdlPosVelEstimator::compareWithGroundTruth(comparison_t& result) const {
  result = comparison_t();

  if (!simGetGroundTruth || !_ready) return false;

  double pos[3], quat[4], linvel[3], angvel[3];
  if (!simGetGroundTruth(pos, quat, linvel, angvel)) return false;

  result.true_position = Eigen::Vector3d(pos[0], pos[1], pos[2]);
  result.true_velocity = Eigen::Vector3d(linvel[0], linvel[1], linvel[2]);
  result.true_orientation = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
  result.true_orientation.normalize();

  // The estimate's r is the body frame origin, which coincides with the free
  // joint origin the simulator reports, so the two are directly comparable with
  // no offset. (Note this is not the centre of mass, which the model places a
  // couple of centimetres away.)
  result.position_error = _xhat.head<3>() - result.true_position;
  result.velocity_error = _xhat.segment<3>(3) - result.true_velocity;

  // Attitude error as a rotation vector: the rotation taking truth to estimate.
  if (_orientation) {
    const Eigen::Quaterniond qErr =
        (result.true_orientation.conjugate() * _orientation->getOrientation())
            .normalized();
    const Eigen::AngleAxisd aa(qErr);
    result.attitude_error = aa.axis() * aa.angle();
  }

  result.position_error_norm = result.position_error.norm();
  result.distance_travelled = _distanceTravelled;
  // Filtered at both ends, so the gait's own sway does not shift the endpoint by
  // a centimetre either way.
  result.net_displacement =
      _havePathSample ? (_pathFilt2 - _startTruePos).head<2>().norm() : 0.0;
  // Drift as a fraction of distance travelled is the headline figure, but it is
  // nonsense until the robot has actually gone somewhere: over a few
  // centimetres any fixed offset reads as a huge percentage. Reported as zero
  // below the threshold rather than as a misleading number.
  result.drift_percent = _distanceTravelled > MIN_DRIFT_DISTANCE
                             ? 100.0 * result.position_error_norm / _distanceTravelled
                             : 0.0;
  result.elapsed = _mgr ? _mgr->readTime() - _startTime : 0.0;

  // How far each foot position estimate sits from the foot it is meant to
  // describe. Reported only while the foot is down: off the ground the state is
  // not tracking anything, it is deliberately being let go.
  if (_haveFootTruth)
    for (int i = 0; i < NUM_LEGS; i++)
      result.foothold_error[i] =
          _contacts[i] ? (_xhat.segment<3>(6 + 3 * i) - _footTruePos[i]).norm() : 0.0;

  if (_errorSamples > 0) {
    const double n = (double)_errorSamples;
    result.position_rms = (_sumSqPos / n).cwiseSqrt();
    result.velocity_rms = (_sumSqVel / n).cwiseSqrt();
    result.attitude_rms = (_sumSqAtt / n).cwiseSqrt();
  }

  result.valid = true;
  return true;
}

void MdlPosVelEstimator::_updateGroundTruthStats() {
  if (!_truthEnable || !_comparison.valid) return;

  const comparison_t& s = _comparison;

  _sumSqPos += s.position_error.cwiseProduct(s.position_error);
  _sumSqVel += s.velocity_error.cwiseProduct(s.velocity_error);
  _sumSqAtt += s.attitude_error.cwiseProduct(s.attitude_error);
  _errorSamples++;

  const double now = _mgr->readTime();

  // Path length: horizontal, low pass filtered, and sampled slowly. Summing the
  // raw position every cycle in three dimensions measures how much the body
  // wobbles rather than how far it goes. A trot bounces the trunk vertically and
  // swings it from side to side at the gait frequency, and the contact impacts
  // ring on top of that; all of it lands in the sum. Measured on this robot, the
  // raw figure came out four times the distance actually covered.
  //
  // Projecting to x-y removes the bounce but not the sway, and decimating only
  // aliases it. Filtering does remove it: two poles at a time constant of about
  // one gait cycle attenuate the oscillation by more than an order of magnitude
  // whatever its phase, and what is left is where the robot went.
  if (!_havePathSample) {
    _startTruePos = s.true_position;
    _pathFilt1 = s.true_position;
    _pathFilt2 = s.true_position;
    _lastPathSample = s.true_position;
    _lastPathTime = now;
    _havePathSample = true;
  } else {
    const double a = _dt / (_pathFilterTau + _dt);
    _pathFilt1 += a * (s.true_position - _pathFilt1);
    _pathFilt2 += a * (_pathFilt1 - _pathFilt2);

    if (now - _lastPathTime >= _pathSamplePeriod) {
      _distanceTravelled += (_pathFilt2 - _lastPathSample).head<2>().norm();
      _lastPathSample = _pathFilt2;
      _lastPathTime = now;
    }
  }

  if (_truthReportPeriod > 0.0 && now - _lastReportTime >= _truthReportPeriod) {
    _lastReportTime = now;

    // Recomputed so the reported RMS and drift include the sample just added.
    comparison_t c;
    compareWithGroundTruth(c);

    // Yaw is reported apart from roll and pitch on purpose. Roll and pitch are
    // held by gravity, but nothing here references absolute yaw, so it drifts
    // by design; averaging them into a single attitude number would make a
    // healthy estimate look broken.
    //
    // Mean contact trust is printed alongside because it now governs how much
    // each foot contributes, and it is the first thing needed to interpret
    // everything else on this line. For a trot at the default settings it
    // should sit near the duty factor times (1 - trust_window), around 0.4;
    // well below that means feet are being doubted more than intended.
    double meanTrust[NUM_LEGS] = {0, 0, 0, 0};
    if (_trustSamples > 0)
      for (int i = 0; i < NUM_LEGS; i++)
        meanTrust[i] = _trustSum[i] / (double)_trustSamples;

    _mgr->message(
        "%s t=%.1fs trust=[%.2f %.2f %.2f %.2f] |pos err|=%.4fm (drift %.1f%% of "
        "%.2fm path, net %.2fm) vel rms=[%.4f %.4f %.4f]m/s roll/pitch rms=[%.4f "
        "%.4f]rad yaw err=%.4frad\n",
        POSVELMODULE_NAME, c.elapsed, meanTrust[0], meanTrust[1], meanTrust[2],
        meanTrust[3], c.position_error_norm, c.drift_percent, c.distance_travelled,
        c.net_displacement, c.velocity_rms.x(), c.velocity_rms.y(), c.velocity_rms.z(),
        c.attitude_rms.x(), c.attitude_rms.y(), c.attitude_error.z());

    for (int i = 0; i < NUM_LEGS; i++) _trustSum[i] = 0.0;
    _trustSamples = 0;

    // Error budget, simulation only. The line above says how wrong the estimate
    // is; this one says how much of that the filter could ever have avoided.
    // Every millimetre a planted foot travels is a violation of the measurement
    // model itself, invisible to the filter and not zero mean, so it biases
    // velocity and integrates into position no matter how clean the IMU is.
    if (_haveFootTruth) {
      double slip[NUM_LEGS];
      for (int i = 0; i < NUM_LEGS; i++)
        slip[i] = _stanceCount[i] > 0 ? _slipSum[i] / (double)_stanceCount[i] : 0.0;

      const double agree =
          _contactSamples > 0 ? 100.0 * (double)_contactAgree / (double)_contactSamples
                              : 0.0;

      // Slip variance per unit stance time carries over unchanged from the
      // filter this replaced, because the foot block of Q0 is dt times the
      // identity here too. So this remains directly comparable with
      // foot_process_noise_position, and the gap between them is the headroom a
      // white noise model needs to cover a systematic rearward scrub.
      Eigen::Vector3d qp = Eigen::Vector3d::Zero();
      if (_stanceTimeTotal > 0.0) qp = _slipSumSqAxis / _stanceTimeTotal;

      _mgr->message(
          "%s   slip/stance=[%.1f %.1f %.1f %.1f]mm (max %.1fmm) "
          "foot pos err=[%.4f %.4f %.4f %.4f]m contacts agree %.1f%%\n"
          "%s   implied foot_process_noise_position=[%.2e %.2e %.2e] (body xyz)\n",
          POSVELMODULE_NAME, 1000.0 * slip[0], 1000.0 * slip[1], 1000.0 * slip[2],
          1000.0 * slip[3],
          1000.0 * std::max(std::max(_slipMax[0], _slipMax[1]),
                            std::max(_slipMax[2], _slipMax[3])),
          _comparison.foothold_error[0], _comparison.foothold_error[1],
          _comparison.foothold_error[2], _comparison.foothold_error[3], agree,
          POSVELMODULE_NAME, qp.x(), qp.y(), qp.z());
    }
  }
}

void MdlPosVelEstimator::_updateSlipStats() {
  _haveFootTruth = false;
  if (!_truthEnable || !simGetFootPositions) return;

  double truth[NUM_LEGS][3];
  if (!simGetFootPositions(truth)) return;
  for (int i = 0; i < NUM_LEGS; i++)
    _footTruePos[i] = Eigen::Vector3d(truth[i][0], truth[i][1], truth[i][2]);
  _haveFootTruth = true;

  // Slip is measured over a whole stance rather than per cycle, because that is
  // the quantity the filter is exposed to: the foot state is corrected toward
  // one place at touchdown and held there until liftoff, so what matters is how
  // far the foot has wandered by the time it leaves.
  const double now = _mgr->readTime();
  for (int i = 0; i < NUM_LEGS; i++) {
    if (_contacts[i] && !_slipTracking[i]) {
      _stanceStartPos[i] = _footTruePos[i];
      _stanceStartTime[i] = now;
      _slipTracking[i] = true;
    } else if (!_contacts[i] && _slipTracking[i]) {
      const Eigen::Vector3d d = _footTruePos[i] - _stanceStartPos[i];
      const double slip = d.norm();
      _slipSum[i] += slip;
      if (slip > _slipMax[i]) _slipMax[i] = slip;
      _stanceCount[i]++;
      _slipTracking[i] = false;

      // Resolved in the body frame: a foot slides along the direction of travel
      // far more readily than it sinks, and separating the axes is what makes
      // the number interpretable.
      const Eigen::Vector3d dBody = _Rbw.transpose() * d;
      _slipSumSqAxis += dBody.cwiseProduct(dBody);
      _stanceTimeTotal += now - _stanceStartTime[i];
    }
  }

  // Agreement between whichever detector is configured and the simulator's own
  // contact state. A detector that holds a swinging foot down makes the filter
  // fuse a moving point as a fixed one, which no amount of tuning elsewhere
  // repairs.
  if (simGetFootContacts) {
    bool truthContacts[NUM_LEGS];
    if (simGetFootContacts(truthContacts, nullptr)) {
      for (int i = 0; i < NUM_LEGS; i++) {
        if (_contacts[i] == truthContacts[i]) _contactAgree++;
        _contactSamples++;
      }
    }
  }
}

void MdlPosVelEstimator::_refreshLogBuffers() {
  const Eigen::Vector3d r = _xhat.head<3>();
  const Eigen::Vector3d v = _xhat.segment<3>(3);

  for (int i = 0; i < 3; i++) {
    _logState[i] = r[i];
    _logState[3 + i] = v[i];
    _logState[6 + i] = _vBody[i];
  }

  for (int i = 0; i < NUM_LEGS; i++) {
    const Eigen::Vector3d p = _xhat.segment<3>(6 + 3 * i);
    _logFootholds[3 * i + 0] = p.x();
    _logFootholds[3 * i + 1] = p.y();
    _logFootholds[3 * i + 2] = p.z();
    // Trust rather than the boolean: it is what the filter actually weighted
    // this foot by, so a recorded run carries the quantity that explains the
    // estimate as well as the force needed to re-tune the thresholds.
    _logContacts[i] = _trust[i];
    _logContacts[NUM_LEGS + i] = _footForce[i];
  }

  for (int i = 0; i < DIM; i++) _logCov[i] = _P(i, i);

  if (_comparison.valid) {
    const comparison_t& c = _comparison;
    _logError[0] = c.position_error.x();
    _logError[1] = c.position_error.y();
    _logError[2] = c.position_error.z();
    _logError[3] = c.velocity_error.x();
    _logError[4] = c.velocity_error.y();
    _logError[5] = c.velocity_error.z();
    _logError[6] = c.attitude_error.x();
    _logError[7] = c.attitude_error.y();
    _logError[8] = c.attitude_error.z();
    _logError[9] = c.drift_percent;
    _logError[10] = c.distance_travelled;
    _logError[11] = c.net_displacement;

    for (int i = 0; i < NUM_LEGS; i++) {
      _logSlip[i] = c.foothold_error[i];
      _logSlip[NUM_LEGS + i] =
          _stanceCount[i] > 0 ? _slipSum[i] / (double)_stanceCount[i] : 0.0;
    }
  }
}

// ---------------------------------------------------------------------------
//  Visualization
// ---------------------------------------------------------------------------

void MdlPosVelEstimator::_updateVisualization() {
  if (!_vizEnable || !simDebugClear || !simDebugSphere) return;

  // Per-leg colours, in FL, FR, RL, RR order.
  static const double legColor[4][4] = {{1.0, 0.55, 0.1, 1.0},
                                        {0.2, 0.8, 1.0, 1.0},
                                        {0.9, 0.3, 0.9, 1.0},
                                        {0.5, 1.0, 0.3, 1.0}};
  static const double estColor[4] = {1.0, 0.15, 0.15, 1.0};
  static const double truthColor[4] = {0.15, 0.4, 1.0, 1.0};
  static const double errColor[4] = {1.0, 1.0, 0.2, 1.0};

  simDebugClear();

  const Eigen::Vector3d r = _xhat.head<3>();
  const double est[3] = {r.x(), r.y(), r.z()};
  simDebugSphere(est, _vizMarkerSize, estColor);

  // Ground truth alongside the estimate, with the error vector between them.
  const comparison_t& c = _comparison;
  const bool haveTruth = c.valid;
  if (haveTruth) {
    const double truth[3] = {c.true_position.x(), c.true_position.y(),
                             c.true_position.z()};
    simDebugSphere(truth, _vizMarkerSize * 0.8, truthColor);
    if (simDebugLine && c.position_error_norm > 1e-4)
      simDebugLine(est, truth, _vizMarkerSize * 0.25, errColor);
  }

  // Estimated foot positions, sized and faded by trust rather than by a
  // boolean. Trust is what the filter weights each foot by, so a marker growing
  // through early stance and fading through late stance shows the ramp working.
  for (int i = 0; i < NUM_LEGS; i++) {
    const Eigen::Vector3d p = _xhat.segment<3>(6 + 3 * i);
    const double pos[3] = {p.x(), p.y(), p.z()};
    double rgba[4] = {legColor[i][0], legColor[i][1], legColor[i][2],
                      0.3 + 0.7 * _trust[i]};
    simDebugSphere(pos, _vizMarkerSize * (0.45 + 0.3 * _trust[i]), rgba);
  }

  // Decimated trails of the estimated and true paths, so drift accumulates
  // visibly instead of having to be inferred from a single marker.
  if (_vizTrail && _vizTrailLength > 0) {
    if (++_trailTick >= _vizTrailDecimation) {
      _trailTick = 0;
      _trailEst[_trailHead] = r;
      _trailTrue[_trailHead] = haveTruth ? c.true_position : r;
      _trailHead = (_trailHead + 1) % _vizTrailLength;
      if (_trailCount < _vizTrailLength) _trailCount++;
    }

    for (int i = 0; i < _trailCount; i++) {
      const int slot = (_trailHead - 1 - i + 2 * _vizTrailLength) % _vizTrailLength;
      // Fade with age so the direction of travel is obvious.
      const double alpha = 0.9 * (1.0 - (double)i / (double)_trailCount);

      const double pe[3] = {_trailEst[slot].x(), _trailEst[slot].y(),
                            _trailEst[slot].z()};
      double ce[4] = {estColor[0], estColor[1], estColor[2], alpha};
      simDebugSphere(pe, _vizMarkerSize * 0.25, ce);

      if (haveTruth) {
        const double pt[3] = {_trailTrue[slot].x(), _trailTrue[slot].y(),
                              _trailTrue[slot].z()};
        double ct[4] = {truthColor[0], truthColor[1], truthColor[2], alpha};
        simDebugSphere(pt, _vizMarkerSize * 0.25, ct);
      }
    }
  }
}

// ---------------------------------------------------------------------------
//  Estimate accessors
// ---------------------------------------------------------------------------

bool MdlPosVelEstimator::getBodyPosition(Eigen::Vector3d& pos) const {
  if (!_ready) return false;
  pos = _xhat.head<3>();
  return true;
}

bool MdlPosVelEstimator::getBodyVelocity(Eigen::Vector3d& vel) const {
  if (!_ready) return false;
  vel = _xhat.segment<3>(3);
  return true;
}

bool MdlPosVelEstimator::getBodyVelocityInBody(Eigen::Vector3d& vel) const {
  if (!_ready) return false;
  vel = _vBody;
  return true;
}

bool MdlPosVelEstimator::getFootPosition(int leg, Eigen::Vector3d& pos) const {
  if (!_ready || leg < 0 || leg >= NUM_LEGS) return false;
  pos = _xhat.segment<3>(6 + 3 * leg);
  return true;
}

bool MdlPosVelEstimator::getContactState(int leg) const {
  if (leg < 0 || leg >= NUM_LEGS) return false;
  return _contacts[leg];
}

double MdlPosVelEstimator::getContactTrust(int leg) const {
  if (leg < 0 || leg >= NUM_LEGS) return 0.0;
  return _trust[leg];
}

double MdlPosVelEstimator::getFootForce(int leg) const {
  if (leg < 0 || leg >= NUM_LEGS) return 0.0;
  return _footForce[leg];
}
