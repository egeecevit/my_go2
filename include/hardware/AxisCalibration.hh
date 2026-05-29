#ifndef _AXISCALIBRATION_HH
#define _AXISCALIBRATION_HH

/** \file

  Pure transforms between hardware-raw and controller-absolute joint
  coordinates for a single motor axis.

  Each motor has a polarity (+1 or -1) and an angular offset. The
  controller works in absolute coordinates while the hardware reports
  and accepts raw values. The two transforms are exact inverses, so a
  commanded position reads back unchanged:

    controller = polarity * raw + offset
    raw        = polarity * (controller - offset)

  Velocity and torque carry no offset; pass offset = 0 for those.
*/
namespace AxisCalibration {

// Hardware reading -> controller-absolute value.
inline double toController(double raw, int polarity, double offset) {
  return polarity * raw + offset;
}

// Controller-absolute command -> hardware value.
inline double toHardware(double command, int polarity, double offset) {
  return polarity * (command - offset);
}

}  // namespace AxisCalibration

#endif
