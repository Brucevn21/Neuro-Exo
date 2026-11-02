#include "../Hardware_Interface.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
int main() {
  Hardware_Interface hardware;
  hardware.setAmpEeg(6);
  hardware.setAmpEog(0);
  uint8_t raw[24]{};
  for (int i = 0; i < 8; ++i) raw[i * 3 + 2] = 1;
  raw[0] = raw[1] = raw[2] = 0xff;  // Signed 24-bit -1
  hardware.toVoltage(raw);
  assert(hardware.volts[0] < 0);
  assert(std::abs(hardware.volts[5] / hardware.volts[1] - 24) < 1e-9);
  bool rejected = false;
  try { hardware.setAmpEeg(7); } catch (const std::invalid_argument&) { rejected = true; }
  assert(rejected);
  assert(hardware.getAmpEeg() == 6);
  const Eigen::MatrixXd sample = Eigen::MatrixXd::Ones(1, 8);
  const auto first = hardware.FilterVoltage(sample, 8);
  const auto second = hardware.FilterVoltage(sample, 8);
  assert(first(0, 0) == 1);
  assert(second(0, 0) == 3);  // Both test filters retained their state.
  assert(!hardware.isBluetoothConnected());
  std::cout << "Hardware interface compile/logic check PASS (test driver/filter substitutes).\n";
}
