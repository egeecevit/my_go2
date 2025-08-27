#ifndef _MOTORHW_HH
#define _MOTORHW_HH

/** \file */

#include "rtcore/Hardware.hh"

/** Interface class for motors.

  This class defines the common abstract interface methods and
  definitions for rotary motors in the system. Each method takes an
  "index" argument, which corresponds to different numbered motor axes
  in the system. As per the base Hardware class definition,
  max_index() gives the number of motors that are supported by the
  underlying hardware instantiation.
 */
class MotorHW : public rtcore::Hardware<MotorHW> {
 public:

  /** Tyoe definition that encapsulates commands to the motor
      hardware. Inspired by the "MIT mode" on CubeMARS motors. */
  typedef struct {
    double t = -1;
    double pos = 0; // rad (absolute)
    double vel= 0;  // rad/s
    double tau = 0; // Nm
    double kp = 0;  // Nm/rad
    double kd = 0;  // Nm/(rad/s)
  } cmd_t;

  /** Tyoe definition that encapsulates the current motor state. This
      state structure assumes that the pos variable is in consistent
      absolute angle coordinates. calibrate() must have been
      called. */
  typedef struct {
    double t = -1;   // timestamp
    double pos = 0;  // rad (absolute)
    double vel = 0;  // rad/s
    double tau = 0;  // Nm
    double temp = 0; // deg Celcius
  } state_t;

  typedef enum {
    STATUS_STARTUP,
    STATUS_READY,
    STATUS_ERROR
  } status_t;
  
  /** Enables the motor associated with the indicated axis index */
  virtual void enable(unsigned int ind) = 0;
  /** Disables the motor associated with the indicated axis index */
  virtual void disable(unsigned int ind) = 0;
  /** Queries the motor enable state for the indicated axis index */
  virtual bool isenabled(unsigned int ind) = 0;

  /** Sets the motor command to be used for the indicated axis index */
  virtual void setCommand( unsigned int ind, cmd_t &cmd ) = 0;
  /** Gets the current motor command for the indicated axis index */
  virtual void getCommand( unsigned int ind, cmd_t &cmd ) = 0;

  /** Gets the current motor state for the indicated axis index */
  virtual void getState( unsigned int ind, state_t &state ) = 0;

  /** Computes the necessary internal offset so that the present
      position for the indicated axis index is at the absolute
      position absAngle */
  virtual void calibrate( unsigned int ind, double absAngle ) = 0;

  /** Queries whether the absolute angular position for the indicated
      axis has been calibrated or not */
  virtual bool isCalibrated( unsigned int ind ) = 0;

  /** This method returns the current status of the specified axis */
  virtual status_t getStatus( unsigned int ind ) = 0;

};
  
#endif
