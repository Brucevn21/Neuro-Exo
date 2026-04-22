/* * Under-Voltage Protection Circuit
 * Threshold: 22V
 * Hardware: Arduino Nano 33 BLE, DFR0017 Relay
 */

const int analogPin = A3;  // Updated Input Pin
const int relayPin = 5;    // Updated Relay Pin

// Resistor values for the voltage divider
const float R1 = 101000.0; // 100k Ohms *Nominal value shown
const float R2 = 9900.0;  // 10k Ohms *Nominal value shown

// Voltage settings
const float thresholdVoltage = 22.0; // Lower Voltage limit
const float hysteresis = 0.5; // Prevents relay chatter (re-engages at 22.5V)

void setup() {
  pinMode(relayPin, OUTPUT);
  
  // Start with the relay OFF for safety until voltage is verified
  digitalWrite(relayPin, LOW); 
  
  Serial.begin(9600);
}

void loop() {
  // Read ADC (0 to 1023)
  int rawValue = analogRead(analogPin);
  
  // Convert ADC value to voltage at the pin (3.3V Logic)
  float vOut = (rawValue * 3.3) / 1023.0;
  
  // Calculate original input voltage based on the divider ratio
  // Formula: Vin = Vout * (R1 + R2) / R2
  float vIn = vOut * ((R1 + R2) / R2);

  // Debugging output to Serial Monitor
  Serial.print("Input Voltage: ");
  Serial.print(vIn);
  Serial.println(" V");

  // Logic for Protection
  if (vIn < thresholdVoltage) {
    // Voltage too low! Disconnect the load.
    digitalWrite(relayPin, LOW); 
  } 
  else if (vIn > (thresholdVoltage + hysteresis)) {
    // Voltage is safe and above recovery threshold. Connect load.
    digitalWrite(relayPin, HIGH); 
  }

  delay(500); // Check every half-second
}