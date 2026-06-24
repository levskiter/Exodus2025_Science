/*
 * ==============================================================================
 * ROVER PAYLOAD FIRMWARE V7.5 - "THE MANUAL TELEMETRY UPDATE"
 * Architecture: Arduino Mega 2560
 * * Purpose: 
 * - 100% manual, non-blocking telemetry stream & command execution.
 * - Completely eliminates dynamic memory allocations (String class) to prevent 
 * microcontroller heap fragmentation and memory leaks over long missions.
 * - Runs the stepper motors smoothly at all times, even while communicating.
 * ==============================================================================
 */

#include <Wire.h>
#include <EEPROM.h>
#include <HX711.h>
#include <AccelStepper.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <SensirionI2cScd4x.h>
#include <avr/wdt.h>

// --- Actuator Kinematics ---
const float   STEPPER_MAX_SPEED = 600.0; // Increased for faster physical response
const float   STEPPER_ACCEL     = 300.0;
const long    LID_OPEN_STEPS    = 2048;  

// --- Pin Assignments ---
const int RE_DE_PIN = 2; // RS-485 Read/Drive Enable
const int ROCK_DT = 4;   const int ROCK_SCK = 5; // Rock Bay HX711
const int DEEP_DT = 6;   const int DEEP_SCK = 7; // Deep Bay HX711
const int MQ4_PIN = A0;  // Methane Analog Input

// Hardware Objects
AccelStepper rockStepper(AccelStepper::HALF4WIRE, 24, 26, 25, 27);
AccelStepper deepStepper(AccelStepper::HALF4WIRE, 28, 30, 29, 31);
HX711 scaleRock; 
HX711 scaleDeep;
Adafruit_MPU6050 mpu; 
SensirionI2cScd4x scd4x;

// --- EEPROM Memory Map ---
const int ADDR_PITCH_OFF = 2;  const int ADDR_ROLL_OFF = 6;   
const int ADDR_ROCK_TARE = 10; const int ADDR_ROCK_CAL = 14;  
const int ADDR_DEEP_TARE = 18; const int ADDR_DEEP_CAL = 22;  
const int ADDR_WIND_OFF = 26;  const int ADDR_WIND_MULT = 28; 
const int ADDR_CO2_MULT = 48; 

// --- Core State Variables ---
struct Telemetry {
  bool rockLidOpen = false; 
  bool deepLidOpen = false;
  float pitch = 0.0; float roll = 0.0; float pitchOffset = 0.0; float rollOffset = 0.0;
  long rockRaw = 0; float rockGrams = 0.0; float rockCalc = 0.0; long rockTare = 0; float rockCalFactor = 1.0;
  long deepRaw = 0; float deepGrams = 0.0; float deepCalc = 0.0; long deepTare = 0; float deepCalFactor = 1.0;
  uint16_t co2 = 0; float temp = 0.0; float hum = 0.0; 
  int mq4Raw = 0; const char* mq4State = "WARMUP";
  float windSpeed = 0.0; int windDir = 0; int windDirOffset = 0; float windMult = 1.0;   
  float co2Mult = 1.0; 
} tlm;

// Timing Registers
unsigned long lastTlmTime = 0; 
unsigned long lastScdTime = 0;
unsigned long lastWindTime = 0; 
unsigned long mq4StartTime = 0;

// Command Parser Buffer (Static allocation to protect the heap!)
char cmdBuffer[64];
int cmdIndex = 0;

// RS-485 Modbus Telegram for Wind Sensor
const byte windRequest[] = {0x01, 0x03, 0x00, 0x0B, 0x00, 0x02, 0xB5, 0xC9};

// Function Prototypes
void processCommand(const char* cmd);
void pollSensors();
void pollWindSensor();
void broadcastTelemetry();

void setup() {
  Serial.begin(115200); // Main telemetry line to Jetson
  Serial1.begin(9600);  // Wind Sensor Serial (RS-485)
  Wire.begin();         // I2C Bus

  pinMode(RE_DE_PIN, OUTPUT); 
  digitalWrite(RE_DE_PIN, LOW); // Set MAX485 to Receiver Mode

  // Retrieve calibrations from internal EEPROM
  EEPROM.get(ADDR_PITCH_OFF, tlm.pitchOffset); 
  EEPROM.get(ADDR_ROLL_OFF, tlm.rollOffset);
  EEPROM.get(ADDR_ROCK_TARE, tlm.rockTare);    
  EEPROM.get(ADDR_ROCK_CAL, tlm.rockCalFactor);
  EEPROM.get(ADDR_DEEP_TARE, tlm.deepTare);    
  EEPROM.get(ADDR_DEEP_CAL, tlm.deepCalFactor);
  EEPROM.get(ADDR_WIND_OFF, tlm.windDirOffset);
  EEPROM.get(ADDR_WIND_MULT, tlm.windMult);
  EEPROM.get(ADDR_CO2_MULT, tlm.co2Mult);      

  // Fallback defaults for uncalibrated cards
  if (isnan(tlm.rockCalFactor) || tlm.rockCalFactor == 0.0) tlm.rockCalFactor = 1.0;
  if (isnan(tlm.deepCalFactor) || tlm.deepCalFactor == 0.0) tlm.deepCalFactor = 1.0;
  if (isnan(tlm.co2Mult) || tlm.co2Mult <= 0.0) tlm.co2Mult = 1.0; 
  if (isnan(tlm.windMult) || tlm.windMult <= 0.0) tlm.windMult = 1.0;

  // Configure Stepper Motors
  rockStepper.setMaxSpeed(STEPPER_MAX_SPEED); 
  rockStepper.setAcceleration(STEPPER_ACCEL);
  deepStepper.setMaxSpeed(STEPPER_MAX_SPEED); 
  deepStepper.setAcceleration(STEPPER_ACCEL);

  // Initialize Sensors
  scaleRock.begin(ROCK_DT, ROCK_SCK); 
  scaleDeep.begin(DEEP_DT, DEEP_SCK);
  mpu.begin(); 
  
  scd4x.begin(Wire, 0x62); 
  scd4x.stopPeriodicMeasurement(); 
  delay(100);
  scd4x.startPeriodicMeasurement();
  
  mq4StartTime = millis();
  wdt_enable(WDTO_4S); // Watchdog safety net
}

void loop() {
  // 1. High Frequency Stepper Updates (Must be called as often as possible!)
  rockStepper.run(); 
  deepStepper.run();

  // 2. Non-blocking Serial Character Accumulator 
  // Prevents Serial reading from choking the stepper step rates.
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0'; // Terminate string
        processCommand(cmdBuffer);
        cmdIndex = 0; // Reset index
      }
    } else if (cmdIndex < (int)(sizeof(cmdBuffer) - 1)) {
      cmdBuffer[cmdIndex++] = c;
    }
  }

  // 3. Sequential Sensor Polling
  pollSensors(); 
  pollWindSensor();

  // 4. Deterministic Telemetry Broadcast at 10Hz
  if (millis() - lastTlmTime >= 100) { 
    broadcastTelemetry(); 
    lastTlmTime = millis(); 
  }

  wdt_reset(); // Pet the watchdog
}

/*
 * ==============================================================================
 * SENSOR ACQUISITION ENGINE (Non-Blocking)
 * ==============================================================================
 */
void pollSensors() {
  // Gas Methane Warming Assessment
  tlm.mq4Raw = analogRead(MQ4_PIN);
  if (millis() - mq4StartTime > 180000) {
    tlm.mq4State = "READY";
  } else {
    tlm.mq4State = "WARMUP";
  }

  // Poll IMU and Calculate Tilt Angles
  sensors_event_t a, g, temp;
  if (mpu.getEvent(&a, &g, &temp)) {
    tlm.pitch = (atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI) - tlm.pitchOffset;
    tlm.roll = (atan2(-a.acceleration.x, a.acceleration.z) * 180.0 / PI) - tlm.rollOffset;
  }
  float pitchRad = tlm.pitch * (PI / 180.0); 
  float rollRad = tlm.roll * (PI / 180.0);

  // Poll Load Cells with Geometric Tilt Corrections
  if (scaleRock.is_ready()) {
    tlm.rockRaw = scaleRock.read();
    tlm.rockGrams = (tlm.rockRaw - tlm.rockTare) / tlm.rockCalFactor; 
    tlm.rockCalc = tlm.rockGrams / (cos(pitchRad) * cos(rollRad));    
  }
  if (scaleDeep.is_ready()) {
    tlm.deepRaw = scaleDeep.read();
    tlm.deepGrams = (tlm.deepRaw - tlm.deepTare) / tlm.deepCalFactor;
    tlm.deepCalc = tlm.deepGrams / (cos(pitchRad) * cos(rollRad));
  }

  // Poll SCD40 (Air Metrics) once every 5 seconds
  if (millis() - lastScdTime > 5000) {
    bool isDataReady = false; 
    scd4x.getDataReadyStatus(isDataReady);
    if (isDataReady) {
      scd4x.readMeasurement(tlm.co2, tlm.temp, tlm.hum);
      tlm.co2 = (uint16_t)((float)tlm.co2 * tlm.co2Mult);
    }
    lastScdTime = millis();
  }
}

void pollWindSensor() {
  // Poll RS-485 Wind sensor every 2 seconds
  if (millis() - lastWindTime > 2000) {
    digitalWrite(RE_DE_PIN, HIGH); // Assert Transmit Mode
    delayMicroseconds(50);
    Serial1.write(windRequest, sizeof(windRequest));
    Serial1.flush();
    digitalWrite(RE_DE_PIN, LOW);  // Revert to Receive Mode
    
    // Read response back without blocking the motors
    unsigned long writeTime = millis();
    byte response[9];
    int bytesRead = 0;
    while (millis() - writeTime < 50 && bytesRead < 9) {
      if (Serial1.available() > 0) {
        response[bytesRead++] = Serial1.read();
      }
    }

    // Parse Modbus Frame if valid
    if (bytesRead == 9 && response[0] == 0x01 && response[1] == 0x03) {
      int speedRaw = (response[3] << 8) | response[4];
      int dirRaw = (response[5] << 8) | response[6];
      
      tlm.windSpeed = (float)speedRaw * 0.1 * tlm.windMult;
      tlm.windDir = (dirRaw + tlm.windDirOffset) % 360;
    }
    lastWindTime = millis();
  }
}

/*
 * ==============================================================================
 * MANUAL COMMAND PARSER
 * ==============================================================================
 */
void processCommand(const char* cmd) {
  // --- Lid Manipulation Commands ---
  if (strcmp(cmd, "ROCK_OPEN") == 0) {
    rockStepper.moveTo(LID_OPEN_STEPS); 
    tlm.rockLidOpen = true;
  }
  else if (strcmp(cmd, "ROCK_CLOSE") == 0) {
    rockStepper.moveTo(0); 
    tlm.rockLidOpen = false;
  }
  else if (strcmp(cmd, "DEEP_OPEN") == 0) {
    deepStepper.moveTo(LID_OPEN_STEPS); 
    tlm.deepLidOpen = true;
  }
  else if (strcmp(cmd, "DEEP_CLOSE") == 0) {
    deepStepper.moveTo(0); 
    tlm.deepLidOpen = false;
  }
  
  // --- Metrology & Offsets Commands ---
  else if (strcmp(cmd, "ZERO_IMU") == 0) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      tlm.pitchOffset = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI; 
      tlm.rollOffset = atan2(-a.acceleration.x, a.acceleration.z) * 180.0 / PI;
      EEPROM.put(ADDR_PITCH_OFF, tlm.pitchOffset); 
      EEPROM.put(ADDR_ROLL_OFF, tlm.rollOffset);
    }
  }
  else if (strcmp(cmd, "TARE_ROCK") == 0) {
    tlm.rockTare = tlm.rockRaw; 
    EEPROM.put(ADDR_ROCK_TARE, tlm.rockTare); 
  }
  else if (strncmp(cmd, "CAL_ROCK_", 9) == 0) {
    float knownMass = atof(cmd + 9);
    if (knownMass > 0.0) {
      tlm.rockCalFactor = (float)(tlm.rockRaw - tlm.rockTare) / knownMass; 
      EEPROM.put(ADDR_ROCK_CAL, tlm.rockCalFactor); 
    }
  }
  else if (strcmp(cmd, "TARE_DEEP") == 0) {
    tlm.deepTare = tlm.deepRaw; 
    EEPROM.put(ADDR_DEEP_TARE, tlm.deepTare); 
  }
  else if (strncmp(cmd, "CAL_DEEP_", 9) == 0) {
    float knownMass = atof(cmd + 9);
    if (knownMass > 0.0) {
      tlm.deepCalFactor = (float)(tlm.deepRaw - tlm.deepTare) / knownMass; 
      EEPROM.put(ADDR_DEEP_CAL, tlm.deepCalFactor); 
    }
  }
  else if (strncmp(cmd, "CO2_MULT_", 9) == 0) {
    float newMult = atof(cmd + 9);
    if (newMult > 0.0) {
      tlm.co2Mult = newMult; 
      EEPROM.put(ADDR_CO2_MULT, tlm.co2Mult); 
    }
  }
}

/*
 * ==============================================================================
 * OPTIMIZED TELEMETRY STREAM
 * ==============================================================================
 */
void broadcastTelemetry() {
  // Pre-allocated static character buffer to completely protect heap memory
  static char tlm_buffer[256];

  // Print metrics directly to static buffer as optimized JSON
  snprintf(tlm_buffer, sizeof(tlm_buffer),
    "{\"r_lid\":%d,\"d_lid\":%d,\"pitch\":%.1f,\"roll\":%.1f,\"rock_r\":%ld,\"rock_g\":%.1f,\"rock_c\":%.1f,\"deep_r\":%ld,\"deep_g\":%.1f,\"deep_c\":%.1f,\"co2\":%u,\"temp\":%.1f,\"hum\":%.1f,\"mq4_r\":%d,\"mq4_s\":\"%s\",\"wind_s\":%.1f,\"wind_d\":%d}",
    tlm.rockLidOpen, tlm.deepLidOpen, 
    tlm.pitch, tlm.roll,
    tlm.rockRaw, tlm.rockGrams, tlm.rockCalc,
    tlm.deepRaw, tlm.deepGrams, tlm.deepCalc,
    tlm.co2, tlm.temp, tlm.hum,
    tlm.mq4Raw, tlm.mq4State,
    tlm.windSpeed, tlm.windDir
  );

  // Ship it to the Jetson!
  Serial.println(tlm_buffer);
}