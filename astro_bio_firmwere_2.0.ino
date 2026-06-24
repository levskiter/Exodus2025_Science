/*
==============================================================================
TAUVER ASTROBIOLOGY SUBSYSTEM V2.0
Hardware: Analog pH Sensor, 30V/10A H-Bridge Motor Driver
Role: Dumb muscle. Reads pH voltage, applies piecewise linear fit, manages 
      bidirectional pump timers, and saves calibrations to EEPROM.
==============================================================================
*/

#include <EEPROM.h>

// --- PINS ---
const int PH_PIN = A3;
const int PUMP_PWM = 3; // Must be a PWM (~) pin for speed control
const int PUMP_DIR = 4; // Digital pin for direction

// --- EEPROM ADDRESSES ---
const int ADDR_PH4  = 0;
const int ADDR_PH7  = 4;
const int ADDR_PH10 = 8;

// --- PH CALIBRATION VARIABLES (Piecewise Linear Fit) ---
float voltageAtPh4  = 3.04; // Default theoretical acidic voltage
float voltageAtPh7  = 2.50; // Default theoretical neutral voltage
float voltageAtPh10 = 1.96; // Default theoretical basic voltage
float currentPh = 7.0;
float currentVoltage = 0.0;

// --- PUMP TIMER VARIABLES ---
unsigned long pumpStartTime = 0;
unsigned long pumpDuration = 0;
bool isPumping = false;
bool isForward = true;

// --- TIMING ---
unsigned long lastSerialTime = 0;

void setup() {
  Serial.begin(115200);
  
  // Configure H-Bridge Pins
  pinMode(PUMP_PWM, OUTPUT);
  pinMode(PUMP_DIR, OUTPUT);
  analogWrite(PUMP_PWM, 0); // Failsafe: Ensure pump is dead on boot

  // Load Calibrations from Memory
  EEPROM.get(ADDR_PH4, voltageAtPh4);
  EEPROM.get(ADDR_PH7, voltageAtPh7);
  EEPROM.get(ADDR_PH10, voltageAtPh10);

  // Sanity check: If EEPROM is blank (new Arduino), reset to defaults
  if (isnan(voltageAtPh4)  || voltageAtPh4 <= 0) voltageAtPh4 = 3.04;
  if (isnan(voltageAtPh7)  || voltageAtPh7 <= 0) voltageAtPh7 = 2.50;
  if (isnan(voltageAtPh10) || voltageAtPh10 <= 0) voltageAtPh10 = 1.96;
}

void loop() {
  unsigned long currentMillis = millis();

  // ====================================================================
  // 1. NON-BLOCKING PUMP MANAGER
  // ====================================================================
  if (isPumping) {
    if (currentMillis - pumpStartTime >= pumpDuration) {
      analogWrite(PUMP_PWM, 0); // Cut power
      isPumping = false;
    }
  }

  // ====================================================================
  // 2. READ & CALCULATE PH (Piecewise Interpolation)
  // ====================================================================
  // Read and smooth the ADC value
  float totalVoltage = 0;
  for(int i=0; i<10; i++) {
    totalVoltage += (analogRead(PH_PIN) * 5.0) / 1023.0;
    delay(2); 
  }
  currentVoltage = totalVoltage / 10.0;

  // The Math: Determine which side of neutral we are on
  float slope = 0;
  if (currentVoltage > voltageAtPh7) {
    // We are in the ACIDIC range (Lower pH = Higher Voltage)
    // Calculate the slope between pH 4 and pH 7
    slope = (voltageAtPh7 - voltageAtPh4) / (7.0 - 4.0); 
  } else {
    // We are in the BASIC range (Higher pH = Lower Voltage)
    // Calculate the slope between pH 7 and pH 10
    slope = (voltageAtPh10 - voltageAtPh7) / (10.0 - 7.0);
  }

  // Apply y = mx + b (rearranged to solve for x)
  if (slope != 0) {
    currentPh = 7.0 + ((currentVoltage - voltageAtPh7) / slope);
  }

  // ====================================================================
  // 3. LISTEN FOR COMMANDS (From USB/Jetson)
  // ====================================================================
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // -- Pump Forward --
    if (cmd.startsWith("PUMP_") && !cmd.startsWith("PUMP_REV_") && cmd != "PUMP_STOP") {
      pumpDuration = cmd.substring(5).toInt();
      pumpStartTime = millis();
      isPumping = true;
      isForward = true;
      digitalWrite(PUMP_DIR, HIGH); // Set forward polarity
      analogWrite(PUMP_PWM, 255);   // 100% Speed
    } 
    // -- Pump Reverse --
    else if (cmd.startsWith("PUMP_REV_")) {
      pumpDuration = cmd.substring(9).toInt();
      pumpStartTime = millis();
      isPumping = true;
      isForward = false;
      digitalWrite(PUMP_DIR, LOW); // Flip polarity
      analogWrite(PUMP_PWM, 255);  // 100% Speed
    }
    // -- Emergency Stop --
    else if (cmd == "PUMP_STOP") {
      isPumping = false;
      analogWrite(PUMP_PWM, 0); // Cut power immediately
    }
    // -- Calibrations --
    else if (cmd == "CAL_4") {
      voltageAtPh4 = currentVoltage;
      EEPROM.put(ADDR_PH4, voltageAtPh4);
    } 
    else if (cmd == "CAL_7") {
      voltageAtPh7 = currentVoltage;
      EEPROM.put(ADDR_PH7, voltageAtPh7);
    }
    else if (cmd == "CAL_10") {
      voltageAtPh10 = currentVoltage;
      EEPROM.put(ADDR_PH10, voltageAtPh10);
    }
  }

  // ====================================================================
  // 4. BLAST JSON TELEMETRY (20Hz)
  // ====================================================================
  if (currentMillis - lastSerialTime >= 50) {
    lastSerialTime = currentMillis;
    
    // Determine the state flag for the GUI
    int stateCode = 0;
    if (isPumping) {
      stateCode = isForward ? 1 : -1;
    }

    Serial.print("{\"ph\":"); Serial.print(currentPh, 2);
    Serial.print(",\"ph_v\":"); Serial.print(currentVoltage, 3);
    Serial.print(",\"pump\":"); Serial.print(stateCode);
    Serial.println("}");
  }
}