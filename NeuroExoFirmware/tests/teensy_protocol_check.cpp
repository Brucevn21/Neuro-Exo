// Exercise the actual Teensy loop/callbacks with inert hardware substitutes.
#include "../src/main.cpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <vector>

using namespace neuroexo;
std::vector<uint8_t> control(Mode mode, int16_t angle = 45) {
  std::vector<uint8_t> bytes(CONTROL_SIZE);
  assert(encodeControl({mode, Speed::High, angle}, bytes.data()));
  return bytes;
}
void receive(const std::vector<uint8_t>& bytes, int reported = -1) {
  Wire.response = bytes;
  Wire.position = 0;
  receiveEvent(reported < 0 ? int(bytes.size()) : reported);
}
void command(const std::vector<uint8_t>& bytes) {
  receive(bytes);
  processI2CCommand();
}
Telemetry readTelemetry() {
  updateTelemetryFrame();
  requestEvent();
  Telemetry value;
  assert(decodeTelemetry(Wire.sent.data(), Wire.sent.size(), value));
  return value;
}
int main() {
  setup();
  assert(!motor.enabled && readTelemetry().has(CommandTimeout));
  assert(!ControlAlgorithm_IsConfigured()); // Calibration is intentionally unset.

  fakeMillis = 10;
  receive(control(Mode::Assistive));
  assert(!motorMotionActive); // Callback only copies; loop applies commands.
  processI2CCommand();
  motorControlISR();
  assert(!motor.enabled && !motorMotionActive);

  // Configure only the host substitute to exercise motion/stop logic.
  auto cfg = makeControlConfig();
  cfg.esconFullScaleCurrentA = 7.0f;
  ControlAlgorithm_SetConfig(cfg);
  command(control(Mode::Neutral));
  command(control(Mode::Assistive));
  assert(motorMotionActive);
  // A repeated heartbeat preserves the trajectory's progress.
  interpInitialized = true;
  interpCounter = 321;
  setPointInterpolated = 12.5f;
  fakeMillis = 100;
  command(control(Mode::Assistive));
  assert(interpInitialized && interpCounter == 321 && setPointInterpolated == 12.5f);
  // Completed trajectories remain complete despite heartbeats.
  motorMotionActive = false;
  interpInitialized = false;
  command(control(Mode::Assistive));
  assert(!motorMotionActive);
  command(control(Mode::Assistive, -123));
  assert(motorMotionActive);
  encDeg = -12;
  measuredMotorCurrentA = -1.2f;
  auto value = readTelemetry();
  assert(value.currentAngleDeg == -12 && value.currentMilliAmps == -1200);
  assert(value.has(MotionActive) && !value.has(CommandTimeout));

  motor.enable();
  command(control(Mode::Neutral, 80));
  assert(!motor.enabled && !motorMotionActive && !readTelemetry().has(MotionActive));

  // Truncation, stale receive-buffer tails, and extra prefix/suffix bytes
  // must never change motor state or refresh the watchdog.
  const auto valid = control(Mode::Assistive);
  std::vector<std::vector<uint8_t>> invalid;
  for (size_t n = 0; n < CONTROL_SIZE; ++n)
    invalid.emplace_back(valid.begin(), valid.begin() + n);
  auto extra = valid; extra.push_back(0); invalid.push_back(extra);
  extra = valid; extra.insert(extra.begin(), 0); invalid.push_back(extra);
  extra = valid; extra.resize(40, 0); invalid.push_back(extra);
  extra = valid; extra[7] ^= 1; invalid.push_back(extra);
  extra = valid; extra[3] = 3; extra[7] = crc8(extra.data()+1, 6); invalid.push_back(extra);
  fakeMillis = 200;
  for (const auto& bytes : invalid) {
    command(bytes);
    assert(invalidCommand && !motorMotionActive && lastValidCommandTime == 100);
  }
  receive({valid.begin(), valid.end()-1}, CONTROL_SIZE);
  processI2CCommand();
  assert(invalidCommand && !motorMotionActive);
  assert(readTelemetry().has(InvalidCommand));

  // Timeout occurs at exactly 250 ms, even with invalid traffic.
  command(valid);
  motor.enable();
  fakeMillis = 449; checkCommandTimeout();
  assert(motorMotionActive && !commandTimedOut);
  fakeMillis = 450; checkCommandTimeout();
  assert(commandTimedOut && !motorMotionActive && !motor.enabled);

  // A fault ignores motion; Neutral still disables output without clearing
  // the latch. Invalid traffic also keeps the fault latched.
  safetyEstopLatched = true;
  command(valid);
  assert(!motorMotionActive && readTelemetry().has(SafetyFault));
  command(control(Mode::Neutral));
  assert(safetyEstopLatched && !motor.enabled);
  fakeMillis = 600; command(extra);
  fakeMillis = 700; checkCommandTimeout();
  assert(commandTimedOut && safetyEstopLatched);
  fakeMillis = 850; checkCommandTimeout();
  assert(!safetyEstopLatched);

  // Unsigned watchdog arithmetic survives millis() wraparound.
  fakeMillis = UINT32_MAX - 99;
  command(valid);
  fakeMillis = 149; checkCommandTimeout();
  assert(!commandTimedOut);
  fakeMillis = 150; checkCommandTimeout();
  assert(commandTimedOut && !motorMotionActive);

  // ControlAlgorithm fault checks run against the imported implementation.
  ControlAlgorithm_SetConfig(cfg);
  NeuroExoControl::ControlInput in{};
  in.dtSec = 0.002f;
  in.assistance = 0.7f;
  in.currentLimitScale = 1;
  in.mode = NeuroExoControl::AssistMode::Assist;
  NeuroExoControl::ControlOutput out;
  in.velocityRadPerSec = cfg.hardVelocityLimitRadPerSec + 1;
  assert(ControlAlgorithm_Update(in, out) == 0);
  assert(ControlAlgorithm_GetEstopReason() == NeuroExoControl::EstopReason::Overspeed);
  ControlAlgorithm_Reset();
  in.velocityRadPerSec = 0;
  in.measuredCurrentA = cfg.overcurrentTripA + 1;
  for (int i = 0; i < cfg.overcurrentTicks; ++i) ControlAlgorithm_Update(in, out);
  assert(ControlAlgorithm_GetEstopReason() == NeuroExoControl::EstopReason::Overcurrent);
  ControlAlgorithm_Reset();
  in.measuredCurrentA = 0;
  in.angleRad = std::numeric_limits<float>::quiet_NaN();
  assert(ControlAlgorithm_Update(in, out) == 0);
  assert(ControlAlgorithm_GetEstopReason() == NeuroExoControl::EstopReason::InvalidInput);
  std::cout << "Teensy: framed commands/telemetry, Neutral stop, heartbeat, watchdog, fault latch PASS\n";
}
