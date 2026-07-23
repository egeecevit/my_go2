#ifndef _IMUHW_HH
#define _IMUHW_HH

/** \file */

#include "rtcore/Hardware.hh"

/** Interface class for IMU sensors.

  This class defines the common abstract interface methods and
  definitions for IMU sensors system. Each method takes an "index"
  argument, which can be used to identify one of possibly multiple
  IMU's attached to the system
 */
class IMUHW : public rtcore::Hardware<IMUHW> {
 public:

  // Type definitions for vectors. Could be replaced by their Eigen
  // counterparts. vec4_t is for quaternion
  typedef struct  { double v[3]; } vec3_t;
  typedef struct  { double v[4]; } vec4_t;
  
  /** Type definition that encapsulates a single IMU reading: raw
      accelerometer, gyro and magnetometer values together with the
      orientation estimate in quaternion and RPY form. */
  typedef struct {
    double t = -1; // t < 0 means invalid reading
    vec3_t acc;    // Raw accelerometer readings
    vec3_t gyro;   // Raw gyro readings
    vec3_t mag;    // Raw magnetometer readings
    vec4_t q;      // Orientation in quaternion form
    double rpy[3] = {0,0,0}; // Orientation in RPY
  } imudata_t;

  typedef enum {
    STATUS_STARTUP,
    STATUS_READY,
    STATUS_ERROR
  } status_t;
  
  /** Gets the latest IMU reading from the indicated sensor index if
      there is data with newer timestamp than the supplied
      data. Returns false if no new data is present, in which case
      "data" will be unmodified.*/
  virtual bool getLastReading( unsigned int ind, imudata_t &data ) = 0;

  /** This method returns the current status of the specified axis */
  virtual status_t getStatus( unsigned int ind ) = 0;

};
  
#endif
