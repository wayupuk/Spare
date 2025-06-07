#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
// #include <time.h> // Included but NTP sync logic not implemented
// #include <esp_sntp.h> // Included but NTP sync logic not implemented
// #include "esp_task_wdt.h" // Included but WDT logic not implemented

#include <Adafruit_ADS1X15.h>
#include <QMI8658.h>
#include <SensorPCF8563.hpp>  // PCF8563 RTC driver

// ──── PIN DEFINITIONS ───────────────────────────────────────────────────────────
#ifndef SENSOR_SCL
#define SENSOR_SCL  14  // I2C Clock pin (Not the typical ESP32 default, but valid)
#endif

#ifndef SENSOR_SDA
#define SENSOR_SDA  15  // I2C Data pin (Not the typical ESP32 default, but valid)
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  4   // PCF8563 interrupt pin (if used by RTC library for alarms)
#endif

// #define WDT_TIMEOUT 3 // Timeout in seconds - WDT not configured in this code

// ──── WIFI & NTP SETTINGS ───────────────────────────────────────────────────────
const char *ssid       = "LoveSugarRain_2.4G";
const char *password   = "0954849012";
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000; // 10 seconds

// ──── PERIPHERAL ADDRESSES & CONSTANTS ─────────────────────────────────────────
const uint8_t ADS1015_1_ADDRESS = 0x48;
const uint8_t ADS1015_2_ADDRESS = 0x49;
const int NUM_FLEX_SENSORS = 5;
const int CALIBRATION_SAMPLES = 500; // Number of samples for sensor calibration

// ──── PERIPHERAL OBJECTS ───────────────────────────────────────────────────────
SensorPCF8563    rtc;
Adafruit_ADS1015 ads1015_1;
Adafruit_ADS1015 ads1015_2;
QMI8658          imu;

// ──── CALIBRATION & FILTER VARIABLES ──────────────────────────────────────────
float accel_offsetX = 0.0f, accel_offsetY = 0.0f, accel_offsetZ = 0.0f;
float gyro_offsetX  = 0.0f, gyro_offsetY  = 0.0f, gyro_offsetZ  = 0.0f;
// bool  calibrated    = false; // Flag is set but not currently used to alter program flow

float mean_acd_max[NUM_FLEX_SENSORS] = {0.0f};
float mean_acd_low[NUM_FLEX_SENSORS] = {0.0f};
float low_value = 0.0f;
float high_value = 1000.0f;

// ──── SENSOR VARIABLES ──────────────────────────────────────────
unsigned long start_timestamp = 0; // For CSV time column
float flex_raw_value[NUM_FLEX_SENSORS];
float flex_cal[NUM_FLEX_SENSORS];

// Simple low-pass filter state
static const float alpha = 0.1f;
float filteredAccelX = 0.0f, filteredAccelY = 0.0f, filteredAccelZ = 0.0f;
float filteredGyroX  = 0.0f, filteredGyroY  = 0.0f, filteredGyroZ  = 0.0f;

// ──── MOTION DETECTION VARIABLES ──────────────────────────────────────────────
static const float GRAVITY         = 9.81f;
static const float accelThreshold  = 2.0f;   // m/s²
static const float gyroThreshold   = 10.0f;  // deg/s
static const float highSq          = (GRAVITY + accelThreshold) * (GRAVITY + accelThreshold);
static const float lowSq           = (GRAVITY - accelThreshold) * (GRAVITY - accelThreshold);
static const float gyroThSq        = gyroThreshold * gyroThreshold;
static const uint32_t MOTION_STOP_TIMEOUT_MS = 2000; // 2 seconds

bool  motionDetected       = false;
unsigned long lastMotionTime = 0;

// ──── PROTOTYPES ────────────────────────────────────────────────────────────────
void flexCalibration();
void gyroCalibration();
void fullCalibration();
void calculateOrientation(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float &roll, float &pitch, float &yaw);
void checkMotionSq(float accelMagSq, float gyroMagSq);
void resetCsvTimestamp(); // Renamed from setTime for clarity
void checkCommand();
void wifiSetup();
void printCountdown(int seconds, const char* message_prefix = "[Arduino]");
void readAllFlexSensors(float* raw_values);
// ────────────────────────────────────────────────────────────────────────────────

void setup() {
  // ─── SERIAL & PERIPHERAL PINS ───────────────────────────────────────────────
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for serial console
  }

  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────┐"));
  Serial.print("[Arduino] "); Serial.println(F("│                 ESP32S3 SKB(Optimized)           │"));
  Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────┘"));
  Serial.println();

  pinMode(SENSOR_IRQ, INPUT_PULLUP); // For PCF8563 interrupt, if used

  // ─── RTC (PCF8563) INITIALIZATION ────────────────────────────────────────────
  // Assuming rtc.begin will initialize the Wire interface with SENSOR_SDA, SENSOR_SCL
  if (!rtc.begin(Wire, SENSOR_SDA, SENSOR_SCL)) {
    Serial.print("[Arduino] "); Serial.println(F("❌ Failed to find PCF8563 RTC. Check wiring!"));
    while (true) {
      delay(1000);
      Serial.print("[Arduino] "); Serial.println(F("⏳ Waiting for PCF8563..."));
    }
  }
  Serial.print("[Arduino] "); Serial.println(F("✅ PCF8563 RTC initialized."));

  // ─── WI-FI SETUP ─────────────────────────────────────────────────────────────
  wifiSetup();

  // ─── ADS1015 INITIALIZATION ─────────────────────────────────────────────────
  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("Initializing ADS1015 #1 (0x48)..."));
  if (!ads1015_1.begin(ADS1015_1_ADDRESS)) { // Will use the Wire instance initialized by RTC
    Serial.print("[Arduino] "); Serial.println(F("❌ Failed to find ADS1015 #1!"));
    while (true) {
      delay(1000);
      Serial.print("[Arduino] "); Serial.println(F("⏳ Waiting for ADS1015 #1..."));
    }
  }
  Serial.print("[Arduino] "); Serial.println(F("✅ ADS1015 #1 OK."));

  Serial.print("[Arduino] "); Serial.println(F("Initializing ADS1015 #2 (0x49)..."));
  if (!ads1015_2.begin(ADS1015_2_ADDRESS)) { // Will use the Wire instance initialized by RTC
    Serial.print("[Arduino] "); Serial.println(F("❌ Failed to find ADS1015 #2!"));
    while (true) {
      delay(1000);
      Serial.print("[Arduino] "); Serial.println(F("⏳ Waiting for ADS1015 #2..."));
    }
  }
  Serial.print("[Arduino] "); Serial.println(F("✅ ADS1015 #2 OK."));

  // ─── QMI8658 IMU INITIALIZATION ───────────────────────────────────────────────
  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("Initializing QMI8658 IMU..."));
  // Assuming imu.begin uses the existing Wire instance or correctly re-initializes with given pins
  if (!imu.begin(SENSOR_SDA, SENSOR_SCL)) {
    Serial.print("[Arduino] "); Serial.println(F("❌ Failed to initialize QMI8658. Check wiring!"));
    while (true) {
      delay(1000);
      Serial.print("[Arduino] "); Serial.println(F("⏳ Waiting for QMI8658..."));
    }
  }
  Serial.print("[Arduino] "); Serial.println(F("✅ QMI8658 initialized."));
  Serial.print("[Arduino] "); Serial.print(F("   WHO_AM_I: 0x"));
  Serial.println(imu.getWhoAmI(), HEX);

  imu.setAccelRange(QMI8658_ACCEL_RANGE_8G);
  imu.setAccelODR(QMI8658_ACCEL_ODR_1000HZ);
  imu.setGyroRange(QMI8658_GYRO_RANGE_512DPS);
  imu.setGyroODR(QMI8658_GYRO_ODR_1000HZ);
  imu.setAccelUnit_mps2(true);
  imu.setGyroUnit_rads(false);  // deg/s
  imu.enableSensors(QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);
  Serial.print("[Arduino] "); Serial.println(F("🔧 QMI8658 sensor configured (8G/512DPS, 1000 Hz)."));

  // ─── CALIBRATION (Optional: call gyroCalibration() here for startup cal)──────
  // gyroCalibration(); // Example: Perform gyro calibration on startup

  // ─── HEADER FOR CSV OUTPUT ────────────────────────────────────────────────────
  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("Time(ms),Motion,Roll,Pitch,Yaw,"
                                                "fAccX,fAccY,fAccZ,fGyroX,fGyroY,fGyroZ,"
                                                "accelMagSq,gyroMagSq," // Added comma
                                                "ADC0,ADC1,ADC2,ADC3,ADC4"));
  resetCsvTimestamp(); // Initialize the time baseline for CSV
}

void loop() {
  // ─── SERIAL COMMAND HANDLING ────────────────────────────────────────────────
  checkCommand();

  // ─── 2) READ ADS1015 ADC CHANNELS ───────────────────────────────────────────
  readAllFlexSensors(flex_raw_value);

  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    // Apply two-point calibration
    if (mean_acd_max[i] != mean_acd_low[i]) { // Avoid division by zero
        flex_cal[i] = ((flex_raw_value[i] - mean_acd_low[i]) * (high_value - low_value) / (mean_acd_max[i] - mean_acd_low[i])) + low_value;
    } else {
        flex_cal[i] = low_value; // or some other default if max and min are same
    }
    // Clamp the values within the reference range
    flex_cal[i] = constrain(flex_cal[i], low_value, high_value);
  }

  // ─── 3) READ & PROCESS IMU ───────────────────────────────────────────────────
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  float accelMagSq = 0.0f, gyroMagSq = 0.0f;

  QMI8658_Data d;
  if (imu.readSensorData(d)) {
    // Apply calibration offsets
    float ax = d.accelX - accel_offsetX;
    float ay = d.accelY - accel_offsetY;
    float az = d.accelZ - accel_offsetZ;

    float gx = d.gyroX  - gyro_offsetX;
    float gy = d.gyroY  - gyro_offsetY;
    float gz = d.gyroZ  - gyro_offsetZ;

    // Low-pass filter
    filteredAccelX = alpha * ax + (1.0f - alpha) * filteredAccelX;
    filteredAccelY = alpha * ay + (1.0f - alpha) * filteredAccelY;
    filteredAccelZ = alpha * az + (1.0f - alpha) * filteredAccelZ;

    filteredGyroX  = alpha * gx + (1.0f - alpha) * filteredGyroX;
    filteredGyroY  = alpha * gy + (1.0f - alpha) * filteredGyroY;
    filteredGyroZ  = alpha * gz + (1.0f - alpha) * filteredGyroZ;

    // Orientation (roll/pitch/yaw)
    calculateOrientation(
      filteredAccelX, filteredAccelY, filteredAccelZ,
      filteredGyroX,  filteredGyroY,  filteredGyroZ,
      roll, pitch, yaw
    );

    // Compute squared magnitudes (using raw, uncalibrated for motion check for simplicity, or calibrated if preferred)
    accelMagSq = (d.accelX * d.accelX) + (d.accelY * d.accelY) + (d.accelZ * d.accelZ); // Using raw for magnitude
    gyroMagSq  = (d.gyroX * d.gyroX) + (d.gyroY * d.gyroY) + (d.gyroZ * d.gyroZ);     // Using raw for magnitude

    checkMotionSq(accelMagSq, gyroMagSq);
  }
  // else: if IMU read fails, keep last filtered values & motion state

  // ─── 4) BATCHED CSV PRINT ────────────────────────────────────────────────────
  static char outBuf[256]; // Increased buffer size slightly just in case
  snprintf(outBuf, sizeof(outBuf),
    "%lu,%d,%.2f,%.2f,%.2f,"   // now, motion, roll, pitch, yaw
    "%.4f,%.4f,%.4f,"         // filteredAccelX, filteredAccelY, filteredAccelZ
    "%.4f,%.4f,%.4f,"         // filteredGyroX, filteredGyroY, filteredGyroZ
    "%.4f,%.4f,"              // accelMagSq, gyroMagSq
    "%.2f,%.2f,%.2f,%.2f,%.2f", // ADC0-4 (calibrated flex)
    millis() - start_timestamp,(motionDetected ? 1 : 0),
    roll, pitch, yaw,
    filteredAccelX, filteredAccelY, filteredAccelZ,
    filteredGyroX, filteredGyroY, filteredGyroZ,
    accelMagSq, gyroMagSq,
    flex_cal[0], flex_cal[1], flex_cal[2], flex_cal[3], flex_cal[4]
  );
  Serial.printf("[Sensor] %s\n", outBuf);

  delay(10); // Simple delay to control loop rate (~100Hz attempt, actual rate depends on processing time + this delay)
}

// -----------------------------------------------------------------------------
// HELPER: Print Countdown
// -----------------------------------------------------------------------------
void printCountdown(int seconds, const char* message_prefix) {
    for (int i = seconds; i > 0; i--) {
        Serial.printf("%s %d...\n", message_prefix, i);
        delay(1000);
    }
}

// -----------------------------------------------------------------------------
// HELPER: Read all flex sensors
// -----------------------------------------------------------------------------
void readAllFlexSensors(float* raw_values) {
    raw_values[0] = ads1015_2.readADC_SingleEnded(0);
    raw_values[1] = ads1015_1.readADC_SingleEnded(0);
    raw_values[2] = ads1015_1.readADC_SingleEnded(1);
    raw_values[3] = ads1015_1.readADC_SingleEnded(2);
    raw_values[4] = ads1015_1.readADC_SingleEnded(3);
}


// -----------------------------------------------------------------------------
// COMMAND HANDLING
// -----------------------------------------------------------------------------
void checkCommand(){
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read until newline
    command.trim();                                // Remove any trailing CR/LF or spaces
    Serial.print("[Arduino] Received: ");
    Serial.println(command);

    if (command.equalsIgnoreCase("help")) {
      Serial.println();
      Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────┐"));
      Serial.print("[Arduino] "); Serial.println(F("│                 Available commands               │"));
      Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────┘"));
      Serial.println();
      Serial.print("[Arduino] "); Serial.println("  help          - Show this help message");
      Serial.print("[Arduino] "); Serial.println("  reset         - Reboot the device");
      Serial.print("[Arduino] "); Serial.println("  gyro_cal      - Gyroscope Calibration");
      Serial.print("[Arduino] "); Serial.println("  flex_cal      - Flex Sensor Calibration");
      Serial.print("[Arduino] "); Serial.println("  full_cal      - Full Calibration (Gyro + Flex)");
      Serial.print("[Arduino] "); Serial.println("  reset_time    - Reset CSV time baseline"); // Clarified command
      Serial.print("[Arduino] "); Serial.println("  setup_wifi    - Re-initialize Wi-Fi connection"); // Clarified
    }
    else if (command.equalsIgnoreCase("reset")) {
      Serial.print("[Arduino] "); Serial.println("➤ Rebooting now…");
      delay(100);             // Give the print a moment to flush
      ESP.restart();
    }
    else if (command.equalsIgnoreCase("gyro_cal")) {
      gyroCalibration();
    }
    else if (command.equalsIgnoreCase("flex_cal")) {
      flexCalibration();
    }
    else if (command.equalsIgnoreCase("reset_time")) { // Matched help
      resetCsvTimestamp();
    }
    else if (command.equalsIgnoreCase("setup_wifi")) { // Matched help
      wifiSetup();
    }
    else if (command.equalsIgnoreCase("full_cal")) {
      fullCalibration();
    } else {
      Serial.print("[Arduino] "); Serial.print("Unknown command: "); Serial.println(command);
    }
  }
}

// -----------------------------------------------------------------------------
// WIFI SETUP
// -----------------------------------------------------------------------------
void wifiSetup(){
  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────┐"));
  Serial.print("[Arduino] "); Serial.println(F("│                 Connecting to Wi-Fi              │"));
  Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────┘"));
  Serial.println();
  Serial.print("[Arduino] "); Serial.print(F("Connecting to Wi-Fi: ")); Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long wifiStart = millis();
  Serial.print("[Arduino] ");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); // Increased delay slightly
    Serial.print(F("."));
    if (millis() - wifiStart > WIFI_CONNECT_TIMEOUT_MS) {
      Serial.println();
      Serial.print("[Arduino] "); Serial.println(F("⚠️  Wi-Fi connect timeout. Retrying..."));
      wifiStart = millis();
      WiFi.disconnect(); // Explicitly disconnect before retrying
      WiFi.begin(ssid, password); // Retry connection
      Serial.print("[Arduino] ");
    }
  }
  Serial.println();
  Serial.print("[Arduino] "); Serial.print(F("✅ Connected to Wi-Fi. IP: "));
  Serial.println(WiFi.localIP());
}

// -----------------------------------------------------------------------------
// CSV TIMESTAMP RESET
// -----------------------------------------------------------------------------
void resetCsvTimestamp(){ // Renamed from setTime
  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────┐"));
  Serial.print("[Arduino] "); Serial.println(F("│                Resetting CSV Timestamp             │"));
  Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────┘"));
  Serial.println();
  start_timestamp = millis();
  Serial.print("[Arduino] "); Serial.println(F("✅ CSV time baseline reset."));
}

// -----------------------------------------------------------------------------
// FLEX SENSOR CALIBRATION
// -----------------------------------------------------------------------------
void flexCalibration() {
  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    mean_acd_max[i] = 0.0f;
    mean_acd_low[i] = 0.0f;
  }

  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────┐"));
  Serial.print("[Arduino] "); Serial.println(F("│           🎯 Starting Flex calibration           │"));
  Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────┘"));
  Serial.println();

  delay(2000); // Shorter initial delay
  Serial.print("[Arduino] "); Serial.println(F("Follow instructions for best calibration results."));
  printCountdown(3, "[Arduino] Flex cal starting in");

  float temp_sensor_values[NUM_FLEX_SENSORS];

  // Two calibration phases: normal hand (0), and fist (1)
  const char* phases[] = {"มือปกติ (normal hand)", "กำหมัด (fist)"};
  for (int phase = 0; phase < 2; phase++) {
    Serial.printf("[Arduino] %s - Hold position. Starting in 5 sec...\n", phases[phase]);
    printCountdown(5, "[Arduino]");

    Serial.printf("[Arduino] 📈 Collecting %d samples for %s:\n", CALIBRATION_SAMPLES, phases[phase]);
    Serial.print("[Arduino] ");

    for (int j = 0; j < CALIBRATION_SAMPLES; j++) {
      readAllFlexSensors(temp_sensor_values);

      for (int k = 0; k < NUM_FLEX_SENSORS; k++) {
        if (phase == 0) { // Normal hand -> Max value (less bend)
          mean_acd_max[k] += temp_sensor_values[k];
        } else { // Fist -> Min value (more bend)
          mean_acd_low[k] += temp_sensor_values[k];
        }
      }
      if ((j % 50) == 0 && j > 0) Serial.print("."); // Progress indicator
      delay(5);
    }
    Serial.println(" Done.");

    // Average calculation
    for (int k = 0; k < NUM_FLEX_SENSORS; k++) {
      if (phase == 0) {
        mean_acd_max[k] /= CALIBRATION_SAMPLES;
      } else {
        mean_acd_low[k] /= CALIBRATION_SAMPLES;
      }
    }
  }

  Serial.println(F("[Arduino] ✅ Flex Sensor Calibration complete."));
  for(int i=0; i<NUM_FLEX_SENSORS; ++i) {
      Serial.printf("[Arduino] Sensor %d: Low Avg: %.2f, High Avg: %.2f\n", i, mean_acd_low[i], mean_acd_max[i]);
  }
}

// -----------------------------------------------------------------------------
// IMU (GYRO/ACCEL) CALIBRATION
// -----------------------------------------------------------------------------
void gyroCalibration() {
  // Static local variables to store sums and counts for calibration
  static float sumAx = 0.0f, sumAy = 0.0f, sumAz = 0.0f;
  static float sumGx = 0.0f, sumGy = 0.0f, sumGz = 0.0f;
  static int validSamples = 0;

  sumAx = sumAy = sumAz = sumGx = sumGy = sumGz = 0.0f; // Reset sums
  validSamples = 0;

  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────┐"));
  Serial.print("[Arduino] "); Serial.println(F("│           🎯 Starting IMU calibration            │"));
  Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────┘"));
  Serial.println();
  delay(2000);

  Serial.print("[Arduino] "); Serial.println(F("Place sensor flat and still. Calibration will start in 5 sec..."));
  printCountdown(5, "[Arduino]");

  Serial.printf("[Arduino] 📈 Collecting %d IMU samples:\n", CALIBRATION_SAMPLES);
  Serial.print("[Arduino] ");

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    QMI8658_Data d;
    if (imu.readSensorData(d)) {
      sumAx += d.accelX;
      sumAy += d.accelY;
      sumAz += d.accelZ;
      sumGx += d.gyroX;
      sumGy += d.gyroY;
      sumGz += d.gyroZ;
      validSamples++;
    }
    if ((i % 50) == 0 && i > 0) Serial.print("."); // Progress indicator
    delay(5);
  }
  Serial.println(" Done.");

  if (validSamples > 0) {
    accel_offsetX = sumAx / validSamples;
    accel_offsetY = sumAy / validSamples;
    accel_offsetZ = (sumAz / validSamples) - GRAVITY;  // Assume Z is aligned with gravity

    gyro_offsetX = sumGx / validSamples;
    gyro_offsetY = sumGy / validSamples;
    gyro_offsetZ = sumGz / validSamples;

    // calibrated = true; // Set flag if needed elsewhere

    Serial.print("[Arduino] "); Serial.println(F("✅ IMU Calibration complete!"));
    Serial.printf("[Arduino]    Accel offsets (m/s²): %.3f, %.3f, %.3f\n", accel_offsetX, accel_offsetY, accel_offsetZ);
    Serial.printf("[Arduino]    Gyro offsets (deg/s):  %.3f, %.3f, %.3f\n", gyro_offsetX, gyro_offsetY, gyro_offsetZ);
  } else {
    Serial.print("[Arduino] "); Serial.println(F("❌ IMU Calibration FAILED. No valid samples collected."));
  }
}

// -----------------------------------------------------------------------------
// FULL CALIBRATION (IMU + FLEX)
// -----------------------------------------------------------------------------
void fullCalibration() {
  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────┐"));
  Serial.print("[Arduino] "); Serial.println(F("│          🎯 Starting Full Calibration            │"));
  Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────┘"));
  Serial.println();
  delay(2000); // Brief pause before starting

  gyroCalibration(); // Calibrate IMU first
  Serial.println(); // Add some spacing
  flexCalibration(); // Then calibrate Flex Sensors

  Serial.print("[Arduino] "); Serial.println(F("✅ Full Calibration Procedure Finished."));
}

// -----------------------------------------------------------------------------
// ORIENTATION CALCULATION
// -----------------------------------------------------------------------------
void calculateOrientation(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float &roll, float &pitch, float &yaw)
{
  // Basic tilt-compensated roll and pitch from accelerometer
  roll  = atan2f(ay, sqrtf(ax*ax + az*az)) * RAD_TO_DEG; // Use RAD_TO_DEG for clarity
  pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;

  // Yaw: simple integration of gz (deg/s * dt) - prone to drift
  static float integratedYaw = 0.0f; // Keep this static for integration
  static uint32_t lastIntegrationTime = 0;

  uint32_t currentTime = millis();
  if (lastIntegrationTime != 0) {
    float dt = (currentTime - lastIntegrationTime) * 0.001f;  // delta time in seconds
    integratedYaw += gz * dt;
  }
  lastIntegrationTime = currentTime;

  // Wrap yaw into [-180, 180] degrees
  // fmodf can be used, or manual wrapping
  while (integratedYaw >  180.0f) integratedYaw -= 360.0f;
  while (integratedYaw < -180.0f) integratedYaw += 360.0f;
  yaw = integratedYaw;
}

// -----------------------------------------------------------------------------
// MOTION DETECTION
// -----------------------------------------------------------------------------
void checkMotionSq(float currentAccelMagSq, float currentGyroMagSq) {
  bool accelMotion = (currentAccelMagSq > highSq) || (currentAccelMagSq < lowSq);
  bool gyroMotion  = (currentGyroMagSq  > gyroThSq);

  if (accelMotion || gyroMotion) {
    // if (!motionDetected) {
      // Serial.print("[Arduino] "); Serial.println(F("🚶 Motion detected!")); // Optional: uncomment for verbose motion start
    // }
    motionDetected = true;
    lastMotionTime = millis();
  } else {
    if (motionDetected && ((millis() - lastMotionTime) > MOTION_STOP_TIMEOUT_MS)) {
      // Serial.print("[Arduino] "); Serial.println(F("🛑 Motion stopped.")); // Optional: uncomment for verbose motion stop
      motionDetected = false;
    }
  }
}