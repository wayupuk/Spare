#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <Adafruit_ADS1X15.h>
#include <QMI8658.h>
#include <SensorPCF8563.hpp>



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

// ──── WIFI & NTP SETTINGS ───────────────────────────────────────────────────────
const char *ssid       = "SURADET";
const char *password   = "0878647861";
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000; // 10 seconds


// ──── PERIPHERAL ADDRESSES & CONSTANTS ─────────────────────────────────────────
const uint8_t ADS1015_1_ADDRESS = 0x48;
const uint8_t ADS1015_2_ADDRESS = 0x49;
const int NUM_FLEX_SENSORS = 5;
const int CALIBRATION_SAMPLES = 1000; // Number of samples for sensor calibration

// ──── PERIPHERAL OBJECTS ───────────────────────────────────────────────────────
SensorPCF8563    rtc;
Adafruit_ADS1015 ads1015_1;
Adafruit_ADS1015 ads1015_2;
QMI8658          imu;


// ──── CALIBRATION & FILTER VARIABLES ──────────────────────────────────────────
float accel_offsetX = 0.0f, accel_offsetY = 0.0f, accel_offsetZ = 0.0f;
float gyro_offsetX  = 0.0f, gyro_offsetY  = 0.0f, gyro_offsetZ  = 0.0f;
float mean_acd_max[NUM_FLEX_SENSORS] = {0.0f};
float mean_acd_low[NUM_FLEX_SENSORS] = {0.0f};
float low_value = 0.0f;
float high_value = 1000.0f;


// ──── SENSOR VARIABLES ──────────────────────────────────────────
unsigned long start_timestamp = 0; // For CSV time column
float flex_raw_value[NUM_FLEX_SENSORS];
float flex_cal[NUM_FLEX_SENSORS];


// ──── MOTION DETECTION VARIABLES ──────────────────────────────────────────────
// static const float GRAVITY         = 9.81f;
// #define RAD_TO_DEG 57.295779513f
static const float GRAVITY         = ONE_G;
// static const float accelThreshold  = 2.0f;   // m/s²
// // static const float gyroThreshold   = 10.0f;  // deg/s
// static const float highSq          = (GRAVITY + accelThreshold) * (GRAVITY + accelThreshold);
// static const float lowSq           = (GRAVITY - accelThreshold) * (GRAVITY - accelThreshold);
// static const float gyroThSq        = gyroThreshold * gyroThreshold;
static const uint32_t MOTION_STOP_TIMEOUT_MS = 2000; // 2 seconds
// Filter variables (simple low-pass filter)
float filteredAccelX = 0, filteredAccelY = 0, filteredAccelZ = 0;
float filteredGyroX = 0, filteredGyroY = 0, filteredGyroZ = 0;
const float alpha = 0.1; // Filter coefficient (0-1, lower = more filtering)


// ──── PROTOTYPES ────────────────────────────────────────────────────────────────
void flexCalibration();
void gyroCalibration();
void fullCalibration();
void calculateOrientation(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float &angle_x, float &angle_y, float &angle_z,
                          float &roll, float &pitch, float &yaw);
void checkMotionSq(float accelMagSq, float gyroMagSq);
void resetCsvTimestamp(); // Renamed from setTime for clarity
void checkCommand();
void wifiSetup();
void printCountdown(int seconds, const char* message_prefix = "[Arduino]");
void readAllFlexSensors(float* raw_values);
void PCF8563_INITIALIZATION();
void ADS1015_INITIALIZATION();
void QMI8658_INITIALIZATION();
void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10); // Wait for serial console
  }
  
  Serial.println();
  Serial.println(F("░██████╗██╗░░██╗██████╗░"));
  Serial.println(F("██╔════╝██║░██╔╝██╔══██╗"));
  Serial.println(F("╚█████╗░█████═╝░██████╦╝"));
  Serial.println(F("░╚═══██╗██╔═██╗░██╔══██╗"));
  Serial.println(F("██████╔╝██║░╚██╗██████╦╝"));
  Serial.println(F("╚═════╝░╚═╝░░╚═╝╚═════╝░"));
  Serial.println();

  Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────────────┐"));
  Serial.print("[Arduino] "); Serial.println(F("│                SKB Co. | Data Collection                 │"));
  Serial.print("[Arduino] "); Serial.println(F("│           Device: ESP32S3 SKB (Optimized Edition)        │"));
  Serial.print("[Arduino] "); Serial.println(F("│        Status: Initialized and Ready to Collect          │"));
  Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────────────┘"));
  Serial.println();

  // ────────────────────────────────────────────────────────────────────────────────
  // pinMode(SENSOR_IRQ, INPUT_PULLUP); // For PCF8563 interrupt, if used

  // ─── Real-time clock/calendar─────────────────────────────────────────────────────────
  PCF8563_INITIALIZATION();
  
  // ─── WI-FI SETUP ─────────────────────────────────────────────────────────────
  wifiSetup();

  // ─── ADS1015 INITIALIZATION ─────────────────────────────────────────────────
  ADS1015_INITIALIZATION();//a 12-bit analog-to-digital converter (ADC) that operates over the I2C protocol

  // ─── QMI8658 INITIALIZATION ─────────────────────────────────────────────────
  QMI8658_INITIALIZATION();

   // ─── HEADER FOR CSV OUTPUT ────────────────────────────────────────────────────
  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("Time(ms),Roll,Pitch,Yaw,"
                                                "RawGyro(deg/s),RawGyro(rad/s)"
                                                "RawAcc(m/s²),"
                                                "ADC0,ADC1,ADC2,ADC3,ADC4"));
  resetCsvTimestamp();
}

void loop() {
  checkCommand();

  // FLEX
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
  
  // ───  READ & PROCESS IMU ───────────────────────────────────────────────────
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f, angle_x = 0.0f, angle_y= 0.0f, angle_z = 0.0f;
  float accelMagSq = 0.0f, gyroMagSq = 0.0f;
  QMI8658_Data d;
  float ax,ay,az,gx,gy,gz;
  if (imu.readSensorData(d)) {
    // Apply calibration offsets
    ax = d.accelX - accel_offsetX;
    ay = d.accelY - accel_offsetY;
    az = d.accelZ - accel_offsetZ;

    gx = d.gyroX  - gyro_offsetX;
    gy = d.gyroY  - gyro_offsetY;
    gz = d.gyroZ  - gyro_offsetZ;

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
      angle_x, angle_y, angle_z,
      roll, pitch, yaw
    );
  }
  // ─── 4) BATCHED CSV PRINT ────────────────────────────────────────────────────
  // "Time(ms),Roll,Pitch,Yaw,RawGyro(deg/s),RawGyro(rad/s),RawAcc(m/s²),ADC0,ADC1,ADC2,ADC3,ADC4"
  static char outBuf[1024]; // Increased buffer size slightly just in case
  // Time(ms),Roll,Pitch,Yaw,
  // RawGyro(deg/s),
  // RawGyro_X(rad/s) ,RawGyro_Y(rad/s) ,RawGyro_Z(rad/s)
  // RawAcc_X(m/s²) ,RawAcc_Y(m/s²) ,RawAcc_Z(m/s²)
  // ADC0,ADC1,ADC2,ADC3,ADC4

  // "Time(ms),
  // RawGyro_X(rad/s),RawGyro_Y(rad/s),RawGyro_Z(rad/s),
  // RawAcc_X(m/s²),RawAcc_Y(m/s²),RawAcc_Z(m/s²),
  // angle_x,angle_y,angle_z
  // ADC0,ADC1,ADC2,ADC3,ADC4"

  snprintf(outBuf, sizeof(outBuf),
    "%lu," 
    "%.4f,%.4f,%.4f,"
    "%.4f,%.4f,%.4f,"
    "%.4f,%.4f,%.4f,"
    "%.4f,%.4f,%.4f,%.4f,%.4f",
    // "Time(ms):%lu ,Roll:%.4f ,Pitch:%.4f ,Yaw:%.4f\n" 
    // "RawGyro_X(rad/s):%.4f ,RawGyro_Y(rad/s):%.4f ,RawGyro_Z(rad/s):%.4f\n"
    // "RawAcc_X(m/s²):%.4f ,RawAcc_Y(m/s²):%.4f ,RawAcc_Z(m/s²):%.4f\n"
    // "angle_x:%.4f ,angle_y:%.4f ,angle_z:%.4f\n"
    // "ADC0:%.4f ,ADC1:%.4f ,ADC2:%.4f ,ADC3:%.4f ,ADC4:%.4f\n",
    millis() - start_timestamp, //,roll, pitch, yaw,
    gx,gy,gz,
    ax,ay,az,
    angle_x,angle_y,angle_z,
    flex_cal[0], flex_cal[1], flex_cal[2], flex_cal[3], flex_cal[4]
  );
  // Serial.printf("%s\n", outBuf);
  Serial.printf("[Sensor] %s\n", outBuf);

  delay(100); // Simple delay to control loop rate (~100Hz attempt, actual rate depends on processing time + this delay)
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
    //   Serial.print("[Arduino] "); Serial.println("  gyro_cal      - Gyroscope Calibration");
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
    // else if (command.equalsIgnoreCase("gyro_cal")) {
    //   gyroCalibration();
    // }
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
// ─── RTC (PCF8563) INITIALIZATION ────────────────────────────────────────────
// Assuming rtc.begin will initialize the Wire interface with SENSOR_SDA, SENSOR_SCL
// -----------------------------------------------------------------------------
void PCF8563_INITIALIZATION(){
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
// ─── ADS1015 INITIALIZATION ─────────────────────────────────────────────────
// -----------------------------------------------------------------------------
void ADS1015_INITIALIZATION(){
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
}

// -----------------------------------------------------------------------------
// ─── QMI8658_INITIALIZATION ─────────────────────────────────────────────────
// -----------------------------------------------------------------------------
void QMI8658_INITIALIZATION(){
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

  imu.setAccelRange(QMI8658_ACCEL_RANGE_2G);
  imu.setAccelODR(QMI8658_ACCEL_ODR_1000HZ);
  imu.setGyroRange(QMI8658_GYRO_RANGE_256DPS);
  imu.setGyroODR(QMI8658_GYRO_ODR_1000HZ);
  imu.setAccelUnit_mps2(true);  // m/s²
  imu.setGyroUnit_rads(true);  // rad/s
  imu.enableSensors(QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);
  Serial.print("[Arduino] "); Serial.println(F("🔧 QMI8658 sensor configured (8G/512DPS, 1000 Hz)."));
  
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
// all this is a peace shit
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// IMU (GYRO/ACCEL) CALIBRATION
// -----------------------------------------------------------------------------
// void gyroCalibration() {
//   // Static local variables to store sums and counts for calibration
//   static float sumAx = 0.0f, sumAy = 0.0f, sumAz = 0.0f;
//   static float sumGx = 0.0f, sumGy = 0.0f, sumGz = 0.0f;
//   static int validSamples = 0;

//   sumAx = sumAy = sumAz = sumGx = sumGy = sumGz = 0.0f; // Reset sums
//   validSamples = 0;

//   Serial.println();
//   Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────┐"));
//   Serial.print("[Arduino] "); Serial.println(F("│           🎯 Starting IMU calibration            │"));
//   Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────┘"));
//   Serial.println();
//   delay(2000);

//   Serial.print("[Arduino] "); Serial.println(F("Place sensor flat and still. Calibration will start in 5 sec..."));
//   printCountdown(5, "[Arduino]");

//   Serial.printf("[Arduino] 📈 Collecting %d IMU samples:\n", CALIBRATION_SAMPLES);
//   Serial.print("[Arduino] ");

//   for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
//     QMI8658_Data d;
//     if (imu.readSensorData(d)) {
//       sumAx += d.accelX;
//       sumAy += d.accelY;
//       sumAz += d.accelZ;
//       sumGx += d.gyroX;
//       sumGy += d.gyroY;
//       sumGz += d.gyroZ;
//       validSamples++;
//     }
//     if ((i % 100) == 0 && i > 0) Serial.print("."); // Progress indicator
//     delay(10);
//   }
//   Serial.println(" Done.");

//   if (validSamples > 0) {
//     accel_offsetX = sumAx / validSamples;
//     accel_offsetY = sumAy / validSamples;
//     accel_offsetZ = (sumAz / validSamples) - GRAVITY;  // Assume Z is aligned with gravity

//     gyro_offsetX = sumGx / validSamples;
//     gyro_offsetY = sumGy / validSamples;
//     gyro_offsetZ = sumGz / validSamples;

//     // calibrated = true; // Set flag if needed elsewhere

//     Serial.print("[Arduino] "); Serial.println(F("✅ IMU Calibration complete!"));
//     Serial.printf("[Arduino]    Accel offsets (m/s²): %.3f, %.3f, %.3f\n", accel_offsetX, accel_offsetY, accel_offsetZ);
//     Serial.printf("[Arduino]    Gyro offsets (rad/s):  %.3f, %.3f, %.3f\n", gyro_offsetX, gyro_offsetY, gyro_offsetZ);
//   } else {
//     Serial.print("[Arduino] "); Serial.println(F("❌ IMU Calibration FAILED. No valid samples collected."));
//   }
// }

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

  // gyroCalibration(); // Calibrate IMU first
  Serial.println(); // Add some spacing
  flexCalibration(); // Then calibrate Flex Sensors

  Serial.print("[Arduino] "); Serial.println(F("✅ Full Calibration Procedure Finished."));
}


// -----------------------------------------------------------------------------
// ORIENTATION CALCULATION
// -----------------------------------------------------------------------------
void calculateOrientation(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float &angle_x, float &angle_y, float &angle_z,
                          float &roll, float &pitch, float &yaw)
{
  // Basic tilt-compensated roll and pitch from accelerometer
  // roll  = atan2f(ay, az) * RAD_TO_DEG;
  // pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  roll  = atan2f(ay, sqrtf(ax*ax + az*az)) * RAD_TO_DEG; // Use RAD_TO_DEG for clarity
  pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;

  angle_x = atan2f(ax, sqrtf(ay*ay + az*az))* RAD_TO_DEG;
  angle_y = atan2f(ay, sqrtf(ax*ax + az*az))* RAD_TO_DEG;
  angle_z = atan2f(sqrtf(ax*ax + ay*ay), az)* RAD_TO_DEG;


  // Yaw: simple integration of gz (deg/s * dt) - prone to drift
  static float integratedYaw = 0.0f; // Keep this static for integration
  static unsigned long lastIntegrationTime = 0;

  unsigned long currentTime = micros(); // Use micros() for higher precision
  if (lastIntegrationTime != 0) {
    // dt is now in seconds. Note the 1.0e-6f for micros -> seconds.
    float dt = (currentTime - lastIntegrationTime) * 1.0e-6f * RAD_TO_DEG; 
    integratedYaw += gz * dt; // Assuming gz is already in degrees/second
  }
  lastIntegrationTime = currentTime;

  yaw = fmodf(integratedYaw, 360.0f);
  if (yaw > 180.0f) {
    yaw -= 360.0f;
  } else if (yaw < -180.0f) {
    yaw += 360.0f;
  }
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