#pragma once
struct motorWiring_t { int enablePin, dirPin, pwmPin, BWSwitchPin, FWSwitchPin; };
struct motorLimit_t { float forwardLimit, backwardLimit; };
struct dir_t { int FORWARD, BACKWARD; };
struct motorDriver {
  bool enabled = false;
  float voltage = 0;
  motorDriver(int, char*, float, int) {}
  void init(motorWiring_t, motorLimit_t) {}
  void disable() { enabled = false; voltage = 0; }
  void enable() { enabled = true; }
  void rotate(float value, dir_t) { voltage = value; }
};
