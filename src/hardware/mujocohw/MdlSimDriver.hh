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

/** \brief Module to interface with MuJoCo simulated quadruped platforms 

  This module implements a simulation driver for MuJoCo-based quadruped
  platforms. Its update() method periodically advances the simulation through
  forward integration after sending joint commands that were set through the
  setJointCommand() method to simulated actuators. Systems states are
  subsequently retrieved from the simulation, and provided through the
  getJointState() and getIMUData() methods.

  Various parameters of this module can be configured through the [simulation]
  configuration table in the ModuleManager database. Please see the example
  simulation.toml for details.
*/
class MdlSimDriver : public rtcore::Module {
public:
  MdlSimDriver();
  virtual ~MdlSimDriver();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  /** \brief Return the joint state for the indicated motor */
  void getJointState( unsigned int ind, MotorHW::state_t &state );

  /** \brief Sets the enable state for the indicated motor */
  void setJointEnabled( unsigned int ind, bool enabled ) {_enabled[ind] = enabled;};
  /** \brief Retrieves the enable state for the indicated motor */
  bool getJointEnabled( unsigned int ind ) { return _enabled[ind]; };

  /** \brief Sets the gainsm the setpoint and the feedforward torque command for
   * the indicated motor */
  void setJointCommand( unsigned int ind, MotorHW::cmd_t &cmd );
  /** \brief Retrieves the current  command for the indicated motor */
  void getJointCommand( unsigned int ind, MotorHW::cmd_t &cmd );
  
  /** \brief Retrieves the current IMU reading from the simulation */
  bool getIMUData( IMUHW::imudata_t &imu );

  /** \brief Retrieves noise-free ground truth for the main body from the free joint.

    Intended for validating estimators against the simulation. pos and linvel are
    expressed in the world frame, quat is the world-from-body rotation in (w,x,y,z)
    order, and angvel is expressed in the body frame (MuJoCo free joint convention,
    the same source used by _readIMUData()). Returns false if the simulation is not
    running or the model has no free joint. Any output pointer may be null. */
  bool getGroundTruth( double pos[3], double quat[4], double linvel[3],
                       double angvel[3] );

  /** \brief Retrieves noise-free foot contact state, indexed by
      QuadrupedKinematics::LegIndex (FL, FR, RL, RR).

    forces receives the normal ground reaction force on each foot in newtons and
    may be null; contacts is simply forces > 0. The force is what makes this
    usable: the model gives every geom a 1mm collision margin, so MuJoCo emits a
    contact record for a foot that is still a millimetre clear of the ground and
    carrying nothing. Reading the record alone would call a foot planted while it
    is still travelling at swing speed. Returns false if the foot geoms could not
    be resolved in the loaded model. */
  bool getFootContacts( bool contacts[4], double forces[4] );

  /** \brief Retrieves the true world position of each foot geom's centre,
      indexed by QuadrupedKinematics::LegIndex.

    This is the centre of the foot sphere, which is exactly the point leg forward
    kinematics returns and therefore exactly what an estimator's foothold state
    represents, so the two are directly comparable with no offset. Returns false
    if the foot geoms could not be resolved. */
  bool getFootPositions( double pos[4][3] );

  // Returns current simulation time in microseconds
  rtcore::CLOCK readClock() { return _now; };

  /** Sets the update period, which is also the integration time step
      between updates */
  void setPeriod( rtcore::CLOCK period ) { _period = period; }

  /** Enable or disable gravity in the simulation */
  void setGravityEnabled(bool enabled) { _gravityFlag = enabled; }
  
  /** Get current gravity state */
  bool isGravityEnabled() const { return _gravityFlag; }

  // Mouse interaction methods
  void _handleMouseButton(int button, int action, int mods);
  void _handleMouseMove(double xpos, double ypos);
  void _handleMouseScroll(double xoffset, double yoffset);

  // Keyboard interaction method
  void _handleKeyboard(int key, int scancode, int action, int mods);


  // MuJoCo controller callback method
  void _controllerCallback(const mjModel* m, mjData* d);

private:
  // Static members for MuJoCo controller callback
  static MdlSimDriver* _instance;
  static void _controllerCallback_static(const mjModel* m, mjData* d);

  rtcore::CLOCK _now = 0;       // Current simulation time in microseconds
  rtcore::CLOCK _period = 1000; // Current integration time step in microseconds
  rtcore::CLOCK _last_render_time = 0;  // Last time rendering was performed
  rtcore::CLOCK _render_interval_us = 40000;  // Render interval in microseconds
  
  // Latest joint state retrieved from the simulation
  MotorHW::state_t _state[12];
  int _polarity[12] = {1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1}; // Polarity for each joint
  
  // Latest command setting relayed by MotorHW
  MotorHW::cmd_t _cmd[12];
  bool _enabled[12] = {false};

  // Torque commands to be sent to simulated joints
  double  _tau[12];

  // Latest IMU data
  IMUHW::imudata_t _imu;
  
  // Gravity control flag
  bool _gravityFlag = true;

  
  // Configured gravity values from TOML (stored for use when _gravityFlag is true)
  double _configured_gravity[3] = {0.0, 0.0, -9.81};
  
  // Headless mode flag (no visualization window)
  bool _headless = false;
  
  void _createSimulation();
  void _destroySimulation();
  
  // Forward integrate the simulation for dt seconds
  void _integrate( double dt );
  
  void _readJointStates();
  void _readIMUData();

  // Appends geoms queued through the simDebug*() interface to the scene. Must be
  // called after mjv_updateScene() (which resets scn.ngeom) and before mjr_render().
  void _renderDebugGeoms();

  // Geom ids of the four feet, resolved once from the model by name. -1 when absent.
  int _footGeomId[4] = {-1, -1, -1, -1};

  mjModel   * _model = nullptr;
  mjData    * _data = nullptr;
  mjvCamera   _cam;  // abstract camera
  mjvOption   _opt;  // visualization options
  mjvScene    _scn;  // abstract scene
  mjrContext  _con;  // custom GPU context
  
  GLFWwindow* _window = nullptr;  // GLFW window for visualization

  // Mouse interaction state
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

};

#endif
