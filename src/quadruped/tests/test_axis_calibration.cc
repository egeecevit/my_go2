#include <cassert>
#include <cmath>
#include <iostream>

#include "hardware/AxisCalibration.hh"

const double TOL = 1e-9;

// A command pushed to hardware and read back must return unchanged, for
// every polarity/offset combination.
void test_command_readback_roundtrip() {
  std::cout << "test_command_readback_roundtrip..." << std::endl;
  double commands[] = {0.0, 0.5, -1.2, 3.14159};
  int polarities[] = {1, -1};
  double offsets[] = {0.0, 0.3, -0.7};
  for (double cmd : commands)
    for (int pol : polarities)
      for (double off : offsets) {
        double raw = AxisCalibration::toHardware(cmd, pol, off);
        double back = AxisCalibration::toController(raw, pol, off);
        if (std::abs(back - cmd) > TOL) {
          std::cerr << "  FAIL cmd=" << cmd << " pol=" << pol << " off=" << off
                    << " -> readback=" << back << std::endl;
          assert(false);
        }
      }
  std::cout << "  PASS" << std::endl;
}

// Reading transform follows the documented convention.
void test_to_controller_values() {
  std::cout << "test_to_controller_values..." << std::endl;
  assert(std::abs(AxisCalibration::toController(2.0, 1, 0.5) - 2.5) < TOL);
  assert(std::abs(AxisCalibration::toController(2.0, -1, 0.5) - (-1.5)) < TOL);
  std::cout << "  PASS" << std::endl;
}

int main() {
  test_command_readback_roundtrip();
  test_to_controller_values();
  std::cout << "All axis calibration tests passed." << std::endl;
  return 0;
}
