/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#ifndef _MDLSIMDRIVER_HH
#define _MDLSIMDRIVER_HH

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include "rtcore/Module.hh"
#include "rtcore/LogServer.hh"
#include "rtcore/ClockHW.hh"

#include "hardware/MotorHW.hh"
#include "hardware/IMUHW.hh"

#define MDLSIMDRIVER_NAME "MdlSimDriver"

/** \brief Module interfacing MuJoCo simulation with the hardware abstractions.

  Each update applies the stored motor commands, advances simulation time, then
  refreshes joint state and IMU data. Configuration is read from the [simulation]
  table. */
class MdlSimDriver : public rtcore::Module {
public:
  MdlSimDriver();
  virtual ~MdlSimDriver();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  void getJointState( unsigned int ind, MotorHW::state_t &state );

  void setJointEnabled( unsigned int ind, bool enabled ) {_enabled[ind] = enabled;};
  bool getJointEnabled( unsigned int ind ) { return _enabled[ind]; };

  void setJointCommand( unsigned int ind, MotorHW::cmd_t &cmd );
  void getJointCommand( unsigned int ind, MotorHW::cmd_t &cmd );
  
  bool getIMUData( IMUHW::imudata_t &imu );

  /** \brief Read noise-free body state from the MuJoCo free joint.

    pos and linvel are world-frame; quat is the world-from-body rotation in
    (w,x,y,z) order; angvel is body-frame. Returns false without a loaded model
    with a free joint. Output pointers may be null. */
  bool getGroundTruth( double pos[3], double quat[4], double linvel[3],
                       double angvel[3] );

  /** \brief Read foot contact state in FL, FR, RL, RR order.

    forces contains normal ground reaction forces [N]; contacts is true when its
    force is positive. Only active contacts at nonpositive distance are counted.
    Returns false if no foot geom was resolved. */
  bool getFootContacts( bool contacts[4], double forces[4] );

  /** \brief Read full world-frame ground reaction for each foot in FL, FR, RL, RR
      order; each vector has normal and tangential components [N].

    Airborne feet are zero. Active contacts within the geom margin are retained;
    contacts with con.exclude != 0 are skipped. MuJoCo's contact-frame wrench is
    rotated by the row-major frame transpose and signed according to whether the
    foot is geom1 or geom2. Returns false if no foot geom was resolved. */
  bool getFootGRF( double grf[4][3] );

  /** \brief Read foot-geom centers in world coordinates, in FL, FR, RL, RR order.
      Returns false if no foot geom was resolved. */
  bool getFootPositions( double pos[4][3] );

  /** \brief Inject a loaded model/data pair and resolve its foot geom IDs.
      Ownership of model and data remains with the caller. */
  void setModelForTesting( mjModel* model, mjData* data );

  // Current simulation time [us].
  rtcore::CLOCK readClock() { return _now; };

  /** Set the update period and integration step [us]. */
  void setPeriod( rtcore::CLOCK period ) { _period = period; }

  void setGravityEnabled(bool enabled) { _gravityFlag = enabled; }
  
  bool isGravityEnabled() const { return _gravityFlag; }

  void _handleMouseButton(int button, int action, int mods);
  void _handleMouseMove(double xpos, double ypos);
  void _handleMouseScroll(double xoffset, double yoffset);

  void _handleKeyboard(int key, int scancode, int action, int mods);


  void _controllerCallback(const mjModel* m, mjData* d);

private:
  // Static state for the MuJoCo controller callback.
  static MdlSimDriver* _instance;
  static void _controllerCallback_static(const mjModel* m, mjData* d);

  rtcore::CLOCK _now = 0;       // Current simulation time [us].
  rtcore::CLOCK _period = 1000; // Integration step [us].
  rtcore::CLOCK _last_render_time = 0;  // Last render time [us].
  rtcore::CLOCK _render_interval_us = 40000;  // Render interval [us].
  
  // Latest joint state in motor-index order.
  MotorHW::state_t _state[12];
  int _polarity[12] = {1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1}; // Command/state sign per joint.
  
  MotorHW::cmd_t _cmd[12];
  bool _enabled[12] = {false};

  double  _tau[12];

  IMUHW::imudata_t _imu;
  // Base linear velocity at the previous tick and the tick length, for the
  // finite-difference accelerometer in _readIMUData().
  double _prevBaseVel[3] = {0.0, 0.0, 0.0};
  double _lastDt = 0.001;
  bool _havePrevBaseVel = false;
  
  bool _gravityFlag = true;

  
  // Configured gravity vector [m/s^2], applied when _gravityFlag is true.
  double _configured_gravity[3] = {0.0, 0.0, -9.81};
  
  bool _headless = false;
  
  void _createSimulation();
  void _destroySimulation();
  
  // Integrate the simulation for dt seconds.
  void _integrate( double dt );
  
  void _readJointStates();
  void _readIMUData();

  // Append queued debug geoms after mjv_updateScene() and before mjr_render().
  void _renderDebugGeoms();

  // Foot geom IDs in FL, FR, RL, RR order; -1 when absent.
  int _footGeomId[4] = {-1, -1, -1, -1};

  // Disturbance push on the trunk ([simulation] push_*): a world-frame force
  // held for _pushDuration, fired once at _pushTime (0 = never) and by the P
  // key in the simulation window. _pushUntil is the end of the active push.
  // _pushRamp > 0 raises the force linearly from zero over that long first.
  int _baseBodyId = -1;
  double _pushForce[3] = {0.0, 0.0, 0.0};
  double _pushDuration = 0.1;
  double _pushRamp = 0.0;
  double _pushTime = 0.0;
  double _pushUntil = -1.0;
  bool _pushFired = false;

  mjModel   * _model = nullptr;
  mjData    * _data = nullptr;
  mjvCamera   _cam;
  mjvOption   _opt;
  mjvScene    _scn;
  mjrContext  _con;
  
  GLFWwindow* _window = nullptr;

  bool _button_left = false;
  bool _button_middle = false;
  bool _button_right = false;
  double _lastx = 0;
  double _lasty = 0;

  rtcore::LogServer *_logserver = nullptr;
  rtcore::LogAccessor *_qpos_a = nullptr;
  rtcore::LogAccessor *_qvel_a = nullptr;
  rtcore::LogAccessor *_qacc_a = nullptr;
  rtcore::LogAccessor *_ctrl_a = nullptr;
  rtcore::LogAccessor *_grf_a = nullptr;

};

#endif
