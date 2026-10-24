#pragma once
#include <vector>
#include <string>
// TEST SUBSTITUTE, no I2C device.
class Imu {
 public:
  std::vector<std::string> s;
  void startImu() {}
  void imuSet() {}
  void setSensAcc(int) {}
  void setSensGyr(int) {}
  void collect() { s = {"IMU: 1 2 3 4 5 6"}; }
};
