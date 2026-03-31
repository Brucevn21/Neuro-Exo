# Troubleshooting Guide - PedExoTeensy_5

## Issue: Teensy Stops Sending CAN Messages After Extended Operation

**Date Discovered:** October 17, 2025  
**Firmware Version:** PedExoTeensy_5  
**Target Hardware:** Teensy 3.2 and 4.1  

---

## Symptoms

- System operates normally for 1,000-15,000 message cycles (approximately 30-480 seconds)
- Teensy suddenly stops sending CAN messages
- Serial output freezes
- Current consumption drops from 100mA to 60mA, then returns to 100mA
- System resumes operation after current returns
- Issue persists across multiple Teensy boards (hardware independent)

---

## Root Cause

**Serial.print() calls inside Interrupt Service Routines (ISRs)**

### Technical Explanation:

1. **Timing Conflict:**
   - Motor control ISR executes every 2ms (500 Hz)
   - CAN receive ISR fires every ~32ms (31.25 Hz)
   - Each `Serial.println()` takes 1-10ms to execute
   - ISRs were printing multiple lines per execution

2. **Buffer Overflow:**
   - Teensy Serial TX buffer: 64 bytes
   - ISRs were generating ~100+ bytes per cycle
   - Buffer filled faster than USB could drain (~12 Mbps = 1.5 MB/s theoretical, but much slower in practice)
   - When buffer full, `Serial.print()` blocks waiting for space

3. **Stack Overflow:**
   - ISRs are nested (CAN ISR can interrupt motor ISR)
   - Blocked ISRs accumulate on call stack
   - Eventually stack overflows → system crash/hang

4. **String Memory Allocation:**
   - Code used `(String) "text" + variable` inside ISRs
   - This allocates heap memory dynamically
   - **CRITICAL:** Dynamic memory allocation in ISRs can cause heap corruption

---

## Solution

**Remove ALL Serial.print() calls from ISRs**

### Changes Made:

1. **ISR_canVcUpdate()** - Disabled all Serial prints
2. **motorControlISR()** - Disabled all Serial prints  
3. **main loop** - Disabled high-frequency encoder prints

### Code Pattern - BEFORE (UNSAFE):
```cpp
void motorControlISR() {
    Serial.println((String) "Vc___ " + Vc);  // ❌ BAD: Blocking + heap allocation in ISR
    // ... control logic
}
```

### Code Pattern - AFTER (SAFE):
```cpp
void motorControlISR() {
    // Serial.println((String) "Vc___ " + Vc);  // DISABLED: Serial in ISR can cause crashes
    // ... control logic
}
```

---

## Best Practices for ISRs

### ✅ DO:
- Keep ISRs as short as possible (<100µs ideal)
- Only update volatile variables
- Use atomic operations for shared data
- Set flags for main loop to handle complex operations

### ❌ DON'T:
- **Never** use `Serial.print()` in ISRs
- **Never** allocate memory (`new`, `malloc`, `String` concatenation)
- **Never** use blocking functions (`delay()`, `delayMicroseconds() > 10µs`)
- **Never** call complex library functions
- **Never** use floating-point math unless absolutely necessary

---

## Debugging Without Serial in ISRs

### Alternative 1: LED Heartbeat
```cpp
volatile uint32_t debugCounter = 0;

void motorControlISR() {
    debugCounter++;
    if (debugCounter % 500 == 0) {  // Every 1 second
        digitalWrite(LED, !digitalRead(LED));  // Toggle LED
    }
}
```

### Alternative 2: Ring Buffer for Logging
```cpp
#define LOG_SIZE 256
volatile uint16_t logBuffer[LOG_SIZE];
volatile uint8_t logIndex = 0;

void motorControlISR() {
    logBuffer[logIndex++] = interpCounter;  // Store data
    if (logIndex >= LOG_SIZE) logIndex = 0;
}

void loop() {
    // Print buffer in main loop (safe)
    static uint8_t lastPrinted = 0;
    if (lastPrinted != logIndex) {
        Serial.println(logBuffer[lastPrinted]);
        lastPrinted = (lastPrinted + 1) % LOG_SIZE;
    }
}
```

### Alternative 3: Digital Pin Toggling (Oscilloscope)
```cpp
void motorControlISR() {
    digitalWrite(DEBUG_PIN, HIGH);  // Fast, non-blocking
    // ... ISR code
    digitalWrite(DEBUG_PIN, LOW);
}
// Measure pulse width on oscilloscope
```

---

## Known Issues / Observations

### 1. CAN Message Count Discrepancy
**Observation:**  
- Messages sent vs received gap grows over time
- Example: Started at 100 vs 110, grew to 43,500 vs 43,700
- Consistent ~200 message offset

**Possible Causes:**
- CAN bus message loss (unlikely on short cables)
- External CAN device counting method differs
- Teensy missing some incoming messages during heavy processing
- Not necessarily a problem if system functions correctly

**Recommendation:** Monitor but do not fix unless functional issues arise

### 2. CAN Transmission Timing Variance
**Observation:**  
- Average: 32ms between messages (expected)
- Oscillates: 30ms - 34ms
- ±2ms jitter

**Causes:**
- Normal ISR timing jitter
- CAN bus arbitration delays
- Interrupt priority conflicts
- Processing time variations

**Impact:** Acceptable for this application (±6% variance)

**Recommendation:** Monitor but do not fix unless timing requirements are stricter

---

## Testing Results

### Before Fix:
- **Crash Time:** 30-480 seconds (1,000-15,000 messages)
- **Current Drop:** 100mA → 60mA → 100mA during crash/recovery
- **Reproducibility:** 100% (every test run crashed)

### After Fix:
- **Uptime:** 15+ minutes continuous operation without crashes
- **Current:** Stable at 100mA
- **Message Rate:** Consistent 32ms ±2ms
- **Reproducibility:** 0% crash rate (ongoing testing)

---

## Prevention Checklist

Before adding code to ISRs, ask:
- [ ] Is this operation <100µs?
- [ ] Does this use Serial, print, or I/O?
- [ ] Does this allocate memory?
- [ ] Could this block or delay?
- [ ] Can this be moved to main loop instead?

If any answer is "uncertain" or "yes" to questions 2-4, **DO NOT** put it in an ISR.

---

## Related Files
- `src/main.cpp` - Main firmware
- `lib/motorDriver/` - Motor control library
- `lib/CAN_utils/` - CAN communication utilities

## Hardware Notes
- **PCB:** Custom exoskeleton controller
- **CAN Bus:** 1 Mbps, extended frame format
- **Power Supply:** Monitor for voltage drops <4.5V if issues recur

---

## Contact
**Authors:** Jesus Rodriguez, Jonathan Izquierdo Perez  
**Lab:** Brain Machine Interface Lab, University of Houston  
**Date:** October 17, 2025
