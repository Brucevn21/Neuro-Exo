#include <ArduinoBLE.h>
#include <Wire.h>
#include "commProtocol.h"

using namespace NeuroExoProtocol;

// --- BLE packet-loss test mode ---
// When enabled, incoming control frames are NOT forwarded to the Teensy.
// Instead each frame's targetAngleDeg is treated as a 16-bit sequence number
// (sender sends 0, 1, 2, ... wrapping at 65535) and the Nano tallies received,
// lost, duplicate and out-of-order packets plus arrival timing. Telemetry
// polling and PMS keep running so the radio/CPU load stays realistic.
// Counters reset on connect; a final summary prints on disconnect.
#define BLE_PACKET_TEST 0

// This Nano 33 BLE plays two roles:
//  1. Mediator: BLE peripheral toward the BeagleBone Black (BLE central) <-> I2C
//     master toward the Teensy 4.1 motor controller.
//  2. PMS: under-voltage protection for the shared power rail (relay cutoff).
const int SLAVE_ADDR = 0x08;
const unsigned long TELEMETRY_INTERVAL_MS = 20; // 50 Hz telemetry to the BeagleBone Black

// Command characteristic: BeagleBone Black -> Nano, framed control packet.
BLEService jointService("180C");
BLECharacteristic commandChar("2A56", BLEWrite | BLEWriteWithoutResponse, MAX_FRAME_SIZE);
// Telemetry characteristic: Nano -> BeagleBone Black, framed telemetry packet.
BLECharacteristic telemetryChar("2A57", BLERead | BLENotify, MAX_FRAME_SIZE);

unsigned long lastTelemetryTime = 0;
unsigned long lastCommandByteTime = 0;
FrameParser commandParser;

// --- Power Management System (under-voltage protection) ---
// Ported from PMSFirmware/NeuroExo_PMS_Code.ino so a single Nano 33 BLE can
// both mediate joint traffic and guard the shared power rail.
const int PMS_ANALOG_PIN = A3;
const int PMS_RELAY_PIN = 5;

// Resistor values for the voltage divider
const float PMS_R1 = 101000.0; // 100k Ohms *Nominal value shown
const float PMS_R2 = 9900.0;   // 10k Ohms *Nominal value shown

// Voltage settings
const float PMS_THRESHOLD_VOLTAGE = 22.0;   // Lower Voltage limit
const float PMS_OVERVOLTAGE_CUTOFF = 30.0;  // Upper Voltage limit
const float PMS_HYSTERESIS = 0.5;           // Prevents relay chatter (re-engages between 22.5V and 32.0V)
const unsigned long PMS_CHECK_INTERVAL_MS = 500; // Sample rate for the voltage divider

unsigned long lastPMSCheckTime = 0;

// Samples the power rail through the voltage divider and drives the relay
// with hysteresis so it doesn't chatter near the threshold. Non-blocking so
// it never stalls BLE/I2C mediation, and runs every loop() iteration
// regardless of BLE connection state since it's a safety function.
void checkPowerSupply() {
  if (millis() - lastPMSCheckTime < PMS_CHECK_INTERVAL_MS) {
    return;
  }
  lastPMSCheckTime = millis();

  // Read ADC (0 to 1023)
  int rawValue = analogRead(PMS_ANALOG_PIN);

  // Convert ADC value to voltage at the pin (3.3V Logic)
  float vOut = (rawValue * 3.3) / 1023.0;

  // Calculate original input voltage based on the divider ratio
  // Formula: Vin = Vout * (R1 + R2) / R2
  float vIn = vOut * ((PMS_R1 + PMS_R2) / PMS_R2);

  if (vIn < PMS_THRESHOLD_VOLTAGE) {
    // Voltage too low! Disconnect the load.
    digitalWrite(PMS_RELAY_PIN, LOW);
  } else if (vIn > PMS_OVERVOLTAGE_CUTOFF) {
    // Voltage too high! Disconnect the load.
    digitalWrite(PMS_RELAY_PIN, LOW);
  } else if (vIn > (PMS_THRESHOLD_VOLTAGE + PMS_HYSTERESIS) &&
             vIn < (PMS_OVERVOLTAGE_CUTOFF - PMS_HYSTERESIS)) {
    // Voltage is safe and inside both recovery thresholds. Connect load.
    digitalWrite(PMS_RELAY_PIN, HIGH);
  }
  // Otherwise, hold the current relay state (hysteresis dead zone).
}

const char *modeName(Mode mode) {
  switch (mode) {
    case Mode::Resistive: return "Resistive";
    case Mode::Assistive: return "Assistive";
    case Mode::Neutral: return "Neutral";
    case Mode::Calibrate: return "Calibrate";
    case Mode::Demo: return "Demo";
    case Mode::Home: return "Home";
    default: return "Unknown";
  }
}

const char *speedName(Speed speed) {
  switch (speed) {
    case Speed::Slow: return "Slow";
    case Speed::Medium: return "Medium";
    case Speed::High: return "High";
    default: return "Unknown";
  }
}

void printReceivedCommand(const uint8_t *frame, uint8_t length, const ControlPacket &command) {
  Serial.println("[Bridge] Packet received from host");
  Serial.print("RAW: [");
  for (uint8_t i = 0; i < length; ++i) {
    if (i > 0) {
      Serial.print(", ");
    }
    Serial.print("0x");
    if (frame[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(frame[i], HEX);
  }
  Serial.println("]");
  // Demo and Home pick their own targets.
  if (command.mode != Mode::Demo && command.mode != Mode::Home) {
    Serial.print("Target Angle: ");
    Serial.print(command.targetAngleDeg);
    Serial.println(" deg");
  }
  Serial.print("Mode: ");
  Serial.println(modeName(command.mode));
  if (command.mode == Mode::Neutral || command.mode == Mode::Demo || command.mode == Mode::Home) {
    Serial.print("Speed: ");
    Serial.println(speedName(static_cast<Speed>(command.setting)));
  } else if (command.mode != Mode::Calibrate) {
    Serial.print("Level: ");
    Serial.print(command.setting);
    Serial.print(" (");
    Serial.print(command.setting * 10);
    Serial.println("% assistance)");
  }
}

#if BLE_PACKET_TEST
const unsigned long TEST_PROGRESS_INTERVAL_MS = 1000; // live progress line cadence
const unsigned long TEST_IDLE_REPORT_MS = 2000;       // summary after this long with no packets

// One bit per sequence number (mod 65536) to detect duplicates. Bits are
// cleared as the highest sequence advances, so tests longer than 65536
// packets work as long as reordering stays under ~32k packets.
const uint32_t SEQ_WINDOW = 65536;
uint8_t seenBitmap[SEQ_WINDOW / 8];

struct PacketTestStats {
  bool anyReceived;
  uint32_t highestSeq;      // highest extended (unwrapped) sequence seen
  uint32_t uniqueReceived;  // distinct sequence numbers received
  uint32_t duplicates;
  uint32_t outOfOrder;      // arrived after a higher sequence number (not dups)
  uint32_t staleDropped;    // too far behind to place in the sequence window
  uint32_t totalFrames;     // every valid control frame, including dups
  uint32_t rejectedWrites;  // BLE writes that did not yield a valid control frame
  uint32_t bleWrites;
  uint32_t bytesReceived;
  unsigned long firstUs;
  unsigned long lastUs;
  unsigned long minGapUs;
  unsigned long maxGapUs;
  uint64_t sumGapUs;
  uint32_t gapCount;
  unsigned long lastPacketMs;
};

PacketTestStats testStats;
uint32_t lastProgressFrames = 0;
unsigned long lastProgressTime = 0;
bool idleReportPrinted = true;

inline bool seqSeen(uint32_t seq) {
  uint32_t idx = seq % SEQ_WINDOW;
  return seenBitmap[idx >> 3] & (1 << (idx & 7));
}

inline void markSeq(uint32_t seq, bool value) {
  uint32_t idx = seq % SEQ_WINDOW;
  if (value) {
    seenBitmap[idx >> 3] |= (1 << (idx & 7));
  } else {
    seenBitmap[idx >> 3] &= ~(1 << (idx & 7));
  }
}

void resetTestStats() {
  memset(&testStats, 0, sizeof(testStats));
  memset(seenBitmap, 0, sizeof(seenBitmap));
  testStats.minGapUs = 0xFFFFFFFF;
  lastProgressFrames = 0;
  lastProgressTime = millis();
  idleReportPrinted = true;
}

uint32_t expectedPackets() {
  // Inferred: sender starts at sequence 0, so expected = highest + 1.
  // Packets dropped at the very end of a run cannot be detected this way.
  return testStats.anyReceived ? testStats.highestSeq + 1 : 0;
}

uint32_t lostPackets() {
  uint32_t expected = expectedPackets();
  return expected > testStats.uniqueReceived ? expected - testStats.uniqueReceived : 0;
}

void recordTestPacket(uint16_t seq16) {
  unsigned long nowUs = micros();
  testStats.totalFrames++;
  testStats.lastPacketMs = millis();
  idleReportPrinted = false;

  if (testStats.totalFrames == 1) {
    testStats.firstUs = nowUs;
  } else {
    unsigned long gap = nowUs - testStats.lastUs;
    if (gap < testStats.minGapUs) testStats.minGapUs = gap;
    if (gap > testStats.maxGapUs) testStats.maxGapUs = gap;
    testStats.sumGapUs += gap;
    testStats.gapCount++;
  }
  testStats.lastUs = nowUs;

  // Unwrap the 16-bit sequence against the highest seen so far.
  int64_t seq;
  if (!testStats.anyReceived) {
    seq = seq16;
  } else {
    int16_t delta = (int16_t)(seq16 - (uint16_t)testStats.highestSeq);
    seq = (int64_t)testStats.highestSeq + delta;
  }
  if (seq < 0) {
    testStats.staleDropped++;
    return;
  }

  if (!testStats.anyReceived || seq > testStats.highestSeq) {
    // Clear bits being reused for the new range before marking this one.
    uint32_t from = testStats.anyReceived ? testStats.highestSeq + 1 : 0;
    uint32_t span = (uint32_t)seq - from + 1;
    if (span >= SEQ_WINDOW) {
      memset(seenBitmap, 0, sizeof(seenBitmap));
    } else {
      for (uint32_t s = from; s <= (uint32_t)seq; ++s) {
        markSeq(s, false);
      }
    }
    testStats.highestSeq = (uint32_t)seq;
    testStats.anyReceived = true;
  } else if (seqSeen((uint32_t)seq)) {
    testStats.duplicates++;
    return;
  } else {
    testStats.outOfOrder++;
  }

  markSeq((uint32_t)seq, true);
  testStats.uniqueReceived++;
}

// Runs inside the BLE stack's poll for every write, so back-to-back writes
// processed in a single poll are each counted (written() would only report
// the latest value and hide them as "lost").
void onTestCommandWritten(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t bytes[MAX_FRAME_SIZE];
  int len = characteristic.readValue(bytes, MAX_FRAME_SIZE);
  testStats.bleWrites++;
  if (len > 0) testStats.bytesReceived += len;

  unsigned long now = millis();
  if (now - lastCommandByteTime > FRAME_TIMEOUT_MS) {
    commandParser.reset();
  }
  lastCommandByteTime = now;

  if (len <= 0 || len > MAX_FRAME_SIZE) {
    testStats.rejectedWrites++;
    return;
  }
  for (int i = 0; i < len; ++i) {
    commandParser.push(bytes[i]);
  }
  Frame frame;
  ControlPacket command;
  if (commandParser.takeFrame(frame) && decodeControlFrame(frame, command)) {
    recordTestPacket((uint16_t)command.targetAngleDeg);
  } else {
    testStats.rejectedWrites++;
  }
}

void printTestSummary(const char *label) {
  uint32_t expected = expectedPackets();
  uint32_t lost = lostPackets();
  float lossPct = expected ? (100.0f * lost / expected) : 0.0f;
  float durationS = testStats.totalFrames > 1 ? (testStats.lastUs - testStats.firstUs) / 1e6f : 0.0f;

  Serial.println();
  Serial.print("===== BLE Packet Test: ");
  Serial.print(label);
  Serial.println(" =====");
  Serial.print("Expected (highest seq + 1): "); Serial.println(expected);
  Serial.print("Received (unique):          "); Serial.println(testStats.uniqueReceived);
  Serial.print("Lost:                       "); Serial.println(lost);
  Serial.print("Loss:                       "); Serial.print(lossPct, 3); Serial.println(" %");
  Serial.print("Duplicates:                 "); Serial.println(testStats.duplicates);
  Serial.print("Out-of-order:               "); Serial.println(testStats.outOfOrder);
  Serial.print("Stale (outside window):     "); Serial.println(testStats.staleDropped);
  Serial.print("Rejected writes (bad frame):"); Serial.print(' '); Serial.println(testStats.rejectedWrites);
  Serial.print("BLE writes / bytes:         ");
  Serial.print(testStats.bleWrites); Serial.print(" / "); Serial.println(testStats.bytesReceived);
  Serial.print("Duration:                   "); Serial.print(durationS, 3); Serial.println(" s");
  if (durationS > 0.0f) {
    Serial.print("Throughput:                 ");
    Serial.print(testStats.totalFrames / durationS, 1); Serial.print(" pkt/s, ");
    Serial.print(testStats.bytesReceived / durationS, 1); Serial.println(" B/s");
  }
  if (testStats.gapCount > 0) {
    Serial.print("Inter-arrival min/avg/max:  ");
    Serial.print(testStats.minGapUs); Serial.print(" / ");
    Serial.print((unsigned long)(testStats.sumGapUs / testStats.gapCount)); Serial.print(" / ");
    Serial.print(testStats.maxGapUs); Serial.println(" us");
  }
  Serial.println("==========================================");
}

// Live progress + idle summary; called from the connected loop.
void servicePacketTest() {
  unsigned long now = millis();
  if (now - lastProgressTime >= TEST_PROGRESS_INTERVAL_MS) {
    lastProgressTime = now;
    if (testStats.totalFrames != lastProgressFrames) {
      uint32_t rate = testStats.totalFrames - lastProgressFrames;
      lastProgressFrames = testStats.totalFrames;
      Serial.print("[Test] rx="); Serial.print(testStats.uniqueReceived);
      Serial.print(" expected="); Serial.print(expectedPackets());
      Serial.print(" lost="); Serial.print(lostPackets());
      Serial.print(" dup="); Serial.print(testStats.duplicates);
      Serial.print(" ooo="); Serial.print(testStats.outOfOrder);
      Serial.print(" bad="); Serial.print(testStats.rejectedWrites);
      Serial.print(" rate="); Serial.print(rate); Serial.println(" pkt/s");
    }
  }

  if (!idleReportPrinted && testStats.totalFrames > 0 &&
      now - testStats.lastPacketMs >= TEST_IDLE_REPORT_MS) {
    idleReportPrinted = true;
    printTestSummary("idle (still connected)");
  }
}
#endif

void setup() {
  // Relay defaults OFF for safety until the rail voltage is verified.
  pinMode(PMS_RELAY_PIN, OUTPUT);
  digitalWrite(PMS_RELAY_PIN, LOW);

  Serial.begin(115200);
  Wire.begin();

  if (!BLE.begin()) {
    Serial.println("BLE failed to start!");
    while (1);
  }

  BLE.setLocalName("Nano33BLE_Master");
  BLE.setAdvertisedService(jointService);
  // Request a 7.5-15 ms connection interval (units of 1.25 ms) so 20 ms
  // telemetry notifications aren't batched and dropped by a slower link.
  BLE.setConnectionInterval(6, 12);
  jointService.addCharacteristic(commandChar);
  jointService.addCharacteristic(telemetryChar);
  BLE.addService(jointService);
#if BLE_PACKET_TEST
  commandChar.setEventHandler(BLEWritten, onTestCommandWritten);
#endif
  BLE.advertise();

  Serial.println("System Ready.");
#if BLE_PACKET_TEST
  Serial.println("BLE PACKET TEST MODE: commands are counted, NOT forwarded to the Teensy.");
  Serial.println("Sender: control frames with targetAngleDeg = 0,1,2,... (one frame per write).");
#endif
}

// A trial starts or stops on a single frame, so retry a failed write rather
// than relying on the next packet in a stream.
const uint8_t I2C_SEND_ATTEMPTS = 3;

void forwardCommandToTeensy(const uint8_t *frame, uint8_t length) {
  for (uint8_t attempt = 1; attempt <= I2C_SEND_ATTEMPTS; ++attempt) {
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(frame, length);
    byte error = Wire.endTransmission();
    if (error == 0) {
      return;
    }
    Serial.print("I2C Error: ");
    Serial.print(error);
    Serial.print(" (attempt ");
    Serial.print(attempt);
    Serial.println(")");
  }
}

void forwardStopToTeensy() {
  uint8_t encoded[MAX_FRAME_SIZE];
  uint8_t frameLength = encodeStopFrame(encoded);
  forwardCommandToTeensy(encoded, frameLength);
}

void pollTeensyTelemetry() {
  int received = Wire.requestFrom(SLAVE_ADDR, (int)MAX_FRAME_SIZE);
  if (received <= 0 || received > MAX_FRAME_SIZE) {
    return;
  }

  FrameParser parser;
  for (int i = 0; i < received; ++i) {
    parser.push((uint8_t)Wire.read());
  }
  Frame frame;
  TelemetryPacket telemetry;
  if (!parser.takeFrame(frame) || !decodeTelemetryFrame(frame, telemetry)) {
    return;
  }

  uint8_t encoded[MAX_FRAME_SIZE];
  uint8_t length = encodeTelemetryFrame(telemetry, encoded);
  telemetryChar.writeValue(encoded, length);
}

void loop() {
  // Runs every iteration, connected or not - power protection can't wait on BLE.
  checkPowerSupply();

  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Central connected: ");
    Serial.println(central.address());
    lastTelemetryTime = millis();
#if BLE_PACKET_TEST
    commandParser.reset();
    resetTestStats();
#endif

    while (central.connected()) {
      checkPowerSupply();

#if BLE_PACKET_TEST
      servicePacketTest();
#else
      if (commandChar.written()) {
        uint8_t bytes[MAX_FRAME_SIZE];
        int len = commandChar.readValue(bytes, MAX_FRAME_SIZE);
        unsigned long now = millis();
        if (now - lastCommandByteTime > FRAME_TIMEOUT_MS) {
          commandParser.reset();
        }
        lastCommandByteTime = now;
        if (len > 0 && len <= MAX_FRAME_SIZE) {
          for (int i = 0; i < len; ++i) {
            commandParser.push(bytes[i]);
          }
          Frame frame;
          ControlPacket command;
          if (commandParser.takeFrame(frame)) {
            if (decodeControlFrame(frame, command)) {
              uint8_t encoded[MAX_FRAME_SIZE];
              uint8_t frameLength = encodeControlFrame(command, encoded);
              printReceivedCommand(encoded, frameLength, command);
              forwardCommandToTeensy(encoded, frameLength);
            } else if (isStopFrame(frame)) {
              Serial.println("[Bridge] Stop received from host");
              forwardStopToTeensy();
            }
          }
        }
      }
#endif

      if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryTime += TELEMETRY_INTERVAL_MS;
        pollTeensyTelemetry();
      }
    }

    Serial.println("Central disconnected");
#if BLE_PACKET_TEST
    printTestSummary("final (disconnected)");
#else
    // Trials no longer need a command stream, so a dropped link must end the
    // trial explicitly or the arm keeps moving with no one able to stop it.
    Serial.println("[Bridge] Sending stop to Teensy (link lost)");
    forwardStopToTeensy();
#endif
  }
}
