#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <time.h>
#include <esp_sntp.h>
#include "esp_task_wdt.h"

#include <Adafruit_ADS1X15.h>
#include <QMI8658.h>
#include <SensorPCF8563.hpp>  // PCF8563 RTC driver


// ──── PIN DEFINITIONS ───────────────────────────────────────────────────────────
#ifndef SENSOR_SCL
#define SENSOR_SCL  14  // ESP32 default SCL
#endif

#ifndef SENSOR_SDA
#define SENSOR_SDA  15  // ESP32 default SDA
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  4   // PCF8563 interrupt pin (if used)
#endif

#define WDT_TIMEOUT 3 // Timeout in seconds
// ──── WIFI & NTP SETTINGS ───────────────────────────────────────────────────────
const char *ssid       = "LoveSugarRain_2.4G";
const char *password   = "0954849012";

// ──── PERIPHERAL OBJECTS ───────────────────────────────────────────────────────
SensorPCF8563    rtc;
Adafruit_ADS1015 ads1015_1;
Adafruit_ADS1015 ads1015_2;
QMI8658          imu;

// ──── CALIBRATION & FILTER VARIABLES ──────────────────────────────────────────
float accel_offsetX = 0, accel_offsetY = 0, accel_offsetZ = 0;
float gyro_offsetX  = 0, gyro_offsetY  = 0, gyro_offsetZ  = 0;
bool  calibrated    = false;

float mean_acd_max[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float mean_acd_low[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float low_value = 0.0;
float high_value = 1000.0;
// ──── SENSOR VARIABLES ──────────────────────────────────────────
int16_t adc0, adc1, adc2, adc3, adc4;
int time_count = 0;
unsigned long start_time = 0;
unsigned long start_timestamp = 0;
bool is_timestamp = false;
float flex_raw_value[5];
float flex_cal[5];

// Simple low-pass filter state
static const float alpha = 0.1f;
float filteredAccelX = 0, filteredAccelY = 0, filteredAccelZ = 0;
float filteredGyroX  = 0, filteredGyroY  = 0, filteredGyroZ  = 0;

// ──── MOTION DETECTION VARIABLES ──────────────────────────────────────────────
// Using squared thresholds to avoid sqrt()
static const float GRAVITY       = 9.81f;
static const float accelThreshold = 2.0f;   // m/s²
static const float gyroThreshold  = 10.0f;  // deg/s
static const float highSq        = (GRAVITY + accelThreshold) * (GRAVITY + accelThreshold);
static const float lowSq         = (GRAVITY - accelThreshold) * (GRAVITY - accelThreshold);
static const float gyroThSq      = gyroThreshold * gyroThreshold;

bool  motionDetected       = false;
unsigned long lastMotionTime = 0;

// ──── LOOP TIMER ───────────────────────────────────────────────────────────────
const uint32_t LOOP_PERIOD_MS = 10;  // target ~100 Hz
uint32_t nextLoopTime = 0;

// ──── PROTOTYPES ────────────────────────────────────────────────────────────────
void flexCalibration();
void gyroCalibration();
void calculateOrientation(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float &roll, float &pitch, float &yaw);
void checkMotionSq(float accelMagSq, float gyroMagSq);
void setTime();
void checkCommand();
void wifiSetup();
// ────────────────────────────────────────────────────────────────────────────────

void setup() {
  // ─── SERIAL & WIRE ───────────────────────────────────────────────────────────
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────┐"));
  Serial.print("[Arduino] "); Serial.println(F("│                 ESP32S3 SKB(Optimized)           │"));
  Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────┘"));
  Serial.println();

  // ─── RTC (PCF8563) INITIALIZATION ────────────────────────────────────────────
  pinMode(SENSOR_IRQ, INPUT_PULLUP);
  Wire.begin(SENSOR_SDA, SENSOR_SCL);
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
  if (!ads1015_1.begin(0x48)) {
    Serial.print("[Arduino] "); Serial.println(F("❌ Failed to find ADS1015 #1!"));
    while (true) {
      delay(1000);
      Serial.print("[Arduino] "); Serial.println(F("⏳ Waiting for ADS1015 #1..."));
    }
  }
  Serial.print("[Arduino] "); Serial.println(F("✅ ADS1015 #1 OK."));

  Serial.print("[Arduino] "); Serial.println(F("Initializing ADS1015 #2 (0x49)..."));
  if (!ads1015_2.begin(0x49)) {
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

  // ─── CALIBRATION ───────────────────────────────────────────────────────────────
  // gyroCalibration();

  // ─── HEADER FOR CSV OUTPUT ────────────────────────────────────────────────────
  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("Time(ms),Motion,Roll,Pitch,Yaw,"
                   "fAccX,fAccY,fAccZ,fGyroX,fGyroY,fGyroZ,"
                   "ADC0,ADC1,ADC2,ADC3,ADC4"));
  // Serial.print("[Sensor] "); Serial.println(F("-----------------------------------------------------------------------"));
  setTime();
}

void loop() {
  // ─── SERIAL COMMAND HANDLING ────────────────────────────────────────────────
  checkCommand();

  // ─── 2) READ ADS1015 ADC CHANNELS ───────────────────────────────────────────
  // adc0 = ads1015_2.readADC_SingleEnded(0);
  // adc1 = ads1015_1.readADC_SingleEnded(0);
  // adc2 = ads1015_1.readADC_SingleEnded(1);
  // adc3 = ads1015_1.readADC_SingleEnded(2);
  // adc4 = ads1015_1.readADC_SingleEnded(3);
  flex_raw_value[0] = ads1015_2.readADC_SingleEnded(0);
  flex_raw_value[1] = ads1015_1.readADC_SingleEnded(0);
  flex_raw_value[2] = ads1015_1.readADC_SingleEnded(1);
  flex_raw_value[3] = ads1015_1.readADC_SingleEnded(2);
  flex_raw_value[4] = ads1015_1.readADC_SingleEnded(3);
  for (int i = 0; i < 5; i++) {
    // Apply two-point calibration
    flex_cal[i] = ((flex_raw_value[i] - mean_acd_low[i]) * (high_value - low_value) / (mean_acd_max[i] - mean_acd_low[i])) + low_value;
    // Clamp the values within the reference range
    if (flex_cal[i] < low_value) flex_cal[i] = low_value;
    if (flex_cal[i] > high_value) flex_cal[i] = high_value;
  }

  // ─── 3) READ & PROCESS IMU ───────────────────────────────────────────────────
  float roll = 0, pitch = 0, yaw = 0;
  float accelMagSq = 0, gyroMagSq = 0;

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

    // Compute squared magnitudes (using raw, unfiltered a/g for motion check)
    accelMagSq = ax*ax + ay*ay + az*az;
    gyroMagSq  = gx*gx + gy*gy + gz*gz;

    checkMotionSq(accelMagSq, gyroMagSq);
  }
  // else: if IMU read fails, keep last filtered values & motion state

  // ─── 4) BATCHED CSV PRINT ────────────────────────────────────────────────────
  // Format:
  // Time(ms),Motion,Roll,Pitch,Yaw,
  // fAccX,fAccY,fAccZ,fGyroX,fGyroY,fGyroZ,
  // ADC0,ADC1,ADC2,ADC3,ADC4
  static char outBuf[200];
  int n = snprintf(outBuf, sizeof(outBuf),
    "%lu,%d,%.2f,%.2f,%.2f,"   // now, motion, roll, pitch, yaw
    "%.4f,%.4f,%.4f,"         // filteredAccelX, filteredAccelY, filteredAccelZ
    "%.4f,%.4f,%.4f,"         // filteredGyroX, filteredGyroY, filteredGyroZ
    "%.4f,%.4f,%.4f,%.4f,%.4f",         // ADC0, ADC1, ADC2, ADC3, ADC4
    millis() - start_timestamp,
    (motionDetected ? 1 : 0),
    roll, pitch, yaw,
    filteredAccelX, filteredAccelY, filteredAccelZ,
    filteredGyroX, filteredGyroY, filteredGyroZ,
    flex_cal[0], flex_cal[1], flex_cal[2], flex_cal[3], flex_cal[4]
  );
  if (n > 0) {
    time_count++;
    Serial.print("[Sensor] ");
    Serial.println(outBuf);
    // Serial.print(outBuf);Serial.print(" time_count:");Serial.print(time_count);
    // Serial.print("\n");
  }

  delay(10);
  // if ((millis() - start_time) >= 1000){
  //   // Serial.print("[Arduino] ");
  //   // Serial.print("in 1 s : ");Serial.println(time_count);
  //   time_count = 0;
  //   start_time = millis();
  // }
}

const int numSamples = 500;  // reduced to speed up calibration
float sensor_value[5];

void checkCommand(){
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read until newline
    command.trim();                                // Remove any trailing CR/LF or spaces
    Serial.print("[Arduino] Received: ");
    Serial.println(command);
    if(command == "help"){
      Serial.println();
      Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────┐"));
      Serial.print("[Arduino] "); Serial.println(F("│          [Arduino] ➤ Available commands         │"));
      Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────┘"));
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣾⣷⣄⠀⠀⠀⣀⣤⣤⣤⡀⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣶⠏⠀⠀⣿⠀⢀⡾⠛⠋⠀⣾⣿⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⡏⠀⠀⠀⣿⢀⣾⠁⠀⣰⠆⢹⡿⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠃⣧⠀⠀⢠⡟⢸⡇⠀⣰⠟⠀⣼⠃⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣹⣆⢀⣸⣇⣸⠃⢠⡏⠀⣸⠋⠀⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣤⣴⣶⣶⣶⠾⠟⠛⠉⠉⠉⠈⠉⠉⠛⠁⢾⠁⣴⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣤⣤⣶⣶⠾⠟⠛⠛⣻⣿⣙⡁⠀⠀⢾⣶⣾⣷⣿⣶⣄⠀⠀⠀⠀⠰⢿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⢀⣀⣀⣀⣠⣴⣶⣶⠾⠟⠛⠉⠉⠉⠀⠀⠀⠀⠀⣿⣻⣟⣻⣿⡦⠀⠘⣿⣿⣛⡿⢶⡇⠀⠀⠀⠀⠀⠀⢻⣆⠀⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⣠⣶⣶⣶⣾⣿⣿⣿⣿⣿⣿⣿⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡟⠙⣿⣿⡗⠀⠀⠿⠉⣿⣿⣿⣶⠀⠀⠀⠀⠀⠀⠈⢿⠀⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⠳⣄⣿⡿⠁⠀⠀⠘⢦⣿⣿⠇⠟⠁⠀⠀⠀⠀⠀⠀⣸⡇⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⡇⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⣇⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⣿⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡏⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⢻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣇⡇⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢤⣤⡀⠀⠀⠀⠀⠀⠀⠀⣿⡇⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠈⠙⢿⣿⣿⣿⣿⣿⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣤⡾⠟⠛⠆⠀⠀⠀⠀⠀⢀⢻⡇⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠈⠙⠿⣿⣭⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣤⣴⣶⠾⠟⠋⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣾⠇⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠙⠛⠷⠶⢶⣶⣦⣤⣴⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣌⣿⠀⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠙⠛⠛⠛⠃⠀⠀⠀⠀⠀⠀⠀⣤⣴⣾⣿⣿⣿⣓⠀⠀⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣼⣿⣷⣦⣄⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣠⣤⣶⣾⣟⣯⣽⠟⠋⠀⠉⠳⣄⠀⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣾⢇⠀⠉⠛⠷⣮⣍⣩⡍⢻⡟⠉⣉⢹⡏⠉⣿⣹⣷⣦⣿⠿⠟⠉⠀⠀⠀⠀⠀⠀⠙⣆⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⠏⢸⠇⠀⠀⠀⠀⠀⠉⠉⠛⠛⠛⠛⠛⠛⠛⠋⠉⠉⠀⠀⠀⠀⠀⢠⣠⡶⠀⠀⠀⠀⠘⣧⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⡿⠀⣸⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠟⠁⠀⠀⠀⠀⠀⠘⣆⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣾⠃⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣾⣇⡀⠀⠀⠀⠀⠀⠀⢹⡆⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣾⠀⣾⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢿⣥⢠⣤⠼⠇⠀⠀⠘⣿⡄\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⣽⡄⠈⢿⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣿⠿⠾⠷⠄⠀⠀⠀⢀⣿⠁\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣧⠀⠸⣷⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣤⣾⠋⠀⠀⠀⠀⠀⠀⢰⣾⡿⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⣦⣠⣿⣿⣶⣶⣤⣤⣄⣀⣀⣀⣀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣠⣴⣿⣇⠀⠀⠀⠀⠀⠀⠀⣸⡟⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢻⣿⠀⠉⠛⢿⣿⣯⣿⡟⢿⠻⣿⢻⣿⢿⣿⣿⣿⣿⣿⠿⠟⠹⣟⢷⣄⠀⠀⠀⢀⣼⠟⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣿⣄⠀⠀⠘⢷⣌⡻⠿⣿⣛⣿⣟⣛⣛⣋⣉⣉⣉⣀⡀⠀⠀⠈⠻⢿⣷⣶⣶⢛⣧⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⣏⠀⠀⠀⠀⠹⢯⣟⣛⢿⣿⣽⣅⣀⡀⠀⣀⡀⠀⠀⠀⠠⢦⣀⠰⡦⠀⢸⠀⣏⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢿⡀⠀⠀⠀⠀⠀⠀⠈⠉⢻⣿⡟⠛⠉⠉⠁⠀⠀⠀⠀⠀⠀⠈⠛⠷⠀⣸⠀⣿⡀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣧⠀⠀⠀⠀⠀⠀⠀⠀⠘⣿⣇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⠀⣿⡇⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⢿⠀⠀⢦⡀⡀⠀⠀⠀⠀⢹⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⡄⡏⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡄⠀⠈⠳⣝⠦⢄⠀⠀⠀⣟⣷⠀⠀⠀⣷⣄⠀⠀⠀⠀⠀⠀⠀⠀⣿⡇⡇⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣄⣷⡀⠀⠀⠈⠙⠂⠀⠀⠀⢸⣿⡄⠀⠀⠘⢦⡙⢦⡀⠀⠀⠀⠀⢰⣷⣷⡇⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⡿⢧⣤⣀⡀⠀⠀⠀⠀⠀⠀⢿⣷⣄⠀⠀⠀⠁⠋⠀⠀⠀⠀⠀⢸⣿⣿⣇⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⣷⡀⠈⠉⠛⠛⠛⠛⠛⠛⠛⠛⢿⡍⠛⠳⠶⣶⣤⣤⣤⣤⣤⣤⠼⠟⡟⢿⡇⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣷⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠰⣾⡇⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⣤⣴⣿⣷⣶⣶⣶⣶⣶⣶⣦⣀⣀⣀⣻⡀⠀⠀⠀⣀⣀⠀⡀⠀⠀⠀⢀⣼⣿⠇⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⠟⠉⠁⠀⠀⠈⠻⣿⡆⢹⣯⣽⣿⣿⠟⠋⠙⣿⣶⣿⣿⣿⣿⣾⣿⣿⣿⣟⠋⠉⣇⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⠀⠀⡀⠀⠀⠀⠈⢻⣆⣿⠀⠀⠀⢁⣶⣿⠿⠟⠛⠷⣶⣽⣿⣿⣻⣏⠙⠃⣴⢻⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⣷⣀⠀⠀⠉⠀⠀⠀⠀⠀⢹⣿⠀⣀⣴⣿⠋⠀⠀⠀⠀⠀⠀⠉⠻⣿⣧⣿⢀⣰⣿⣿⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⢿⣶⣶⣤⣤⣤⣤⣤⣤⣾⣿⣟⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣿⣅⣾⢿⣵⠇⠀⠀⠀⠀\n");
      Serial.print(u8"⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠛⠛⠛⠛⠛⠛⠛⠛⠉⠉⠉⠁⢹⣜⠷⠦⠤⠤⠤⠤⠤⠴⠶⠛⣉⣱⠿⠁⠀⠀⠀⠀⠀\n");
      Serial.println();
      Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────┐"));
      Serial.print("[Arduino] "); Serial.println(F("│          [Arduino] ➤ Available commands         │"));
      Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────┘"));
      Serial.println();
      // Serial.println("[Arduino] ➤ Available commands:");
      Serial.print("[Arduino] "); Serial.println("  help          - Show this help message");
      Serial.print("[Arduino] "); Serial.println("  reset         - Reboot the device");
      Serial.print("[Arduino] "); Serial.println("  gyro_cal      - gyro_Calibration");
      Serial.print("[Arduino] "); Serial.println("  flex_cal      - flex_Calibration");
      Serial.print("[Arduino] "); Serial.println("  set_time      - reset time");
      Serial.print("[Arduino] "); Serial.println("  set_wifi      - reset wifi");
    }
    else if (command == "reset") {
      Serial.print("[Arduino] "); Serial.println("➤ Rebooting now…");
      delay(50);             // Give the print a moment to flush
      ESP.restart();         // Software reset
      // (no code after ESP.restart() will run)
    }
    else if(command == "gyro_cal"){
      gyroCalibration();
    }
    else if(command == "flex_cal"){
      flexCalibration();
    }
    else if(command == "set_time"){
      setTime();
    }
    else if(command == "set_wifi"){
      wifiSetup();
    }
    // … other processing for commands like "cal" below …
  }
}

void wifiSetup(){
  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────┐"));
  Serial.print("[Arduino] "); Serial.println(F("│                 Connecting to Wi-Fi:             │"));
  Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────┘"));
  Serial.println();
  // Serial.print("[Arduino] "); Serial.print(F("Connecting to Wi-Fi: "));
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  unsigned long wifiStart = millis();
  Serial.print("[Arduino] ");
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(F("."));
    if (millis() - wifiStart > 10000) {
      Serial.println();
      Serial.print("[Arduino] "); Serial.println(F("⚠️  Wi-Fi connect timeout. Retrying..."));
      wifiStart = millis();
      WiFi.begin(ssid, password);
    }
  }
  Serial.println();
  Serial.print("[Arduino] "); Serial.print(F("✅ Connected to Wi-Fi. IP: "));
  Serial.println(WiFi.localIP());
}

void setTime(){
  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────┐"));
  Serial.print("[Arduino] "); Serial.println(F("│                      SetTime                     │"));
  Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────┘"));
  Serial.println();

  start_timestamp = millis();
}

// ─── PERFORM Flex CALIBRATION ─────────────────────────────────────────────────────
void flexCalibration() {
  // Reset values
  for (int i = 0; i < 5; i++) {
    mean_acd_max[i] = 0.0;
    mean_acd_low[i] = 0.0;
  }

  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────┐"));
  Serial.print("[Arduino] "); Serial.println(F("│           🎯 Starting Flex calibration           │"));
  Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────┘"));
  Serial.println();

  delay(3000);
  Serial.print("[Arduino] "); Serial.println(F("Do what I told you!!!!!!!! for better calibration"));

  for (int i = 5; i > 0; i--) {
    Serial.print("[Arduino] "); Serial.print(i); Serial.println(F("..."));
    delay(1000);
  }

  // Two calibration phases: normal hand (0), and fist (1)
  for (int phase = 0; phase < 2; phase++) {
    if (phase == 0) {
      Serial.print("[Arduino] "); Serial.println(F("มือปกติ (normal hand) - start in 5 sec..."));
    } else {
      Serial.print("[Arduino] "); Serial.println(F("กำหมัด (fist) - start in 5 sec..."));
    }

    for (int i = 5; i > 0; i--) {
      Serial.print("[Arduino] "); Serial.print(i); Serial.println(F("..."));
      delay(1000);
    }

    Serial.print("[Arduino] "); Serial.print(F("📈 Collecting "));
    Serial.print(numSamples); Serial.println(F(" samples:"));

    // Collect samples
    for (int j = 0; j < numSamples; j++) {
      sensor_value[0] = ads1015_2.readADC_SingleEnded(0);
      sensor_value[1] = ads1015_1.readADC_SingleEnded(0);
      sensor_value[2] = ads1015_1.readADC_SingleEnded(1);
      sensor_value[3] = ads1015_1.readADC_SingleEnded(2);
      sensor_value[4] = ads1015_1.readADC_SingleEnded(3);

      for (int k = 0; k < 5; k++) {
        if (phase == 0) {
          mean_acd_max[k] += sensor_value[k];
        } else {
          mean_acd_low[k] += sensor_value[k];
        }
      }

      delay(5); // Optional: small delay between samples
    }

    // Average calculation
    for (int k = 0; k < 5; k++) {
      if (phase == 0) {
        mean_acd_max[k] /= numSamples;
      } else {
        mean_acd_low[k] /= numSamples;
      }
    }
  }

  Serial.println(F("[Arduino] ✅ Calibration complete."));
}

float sumAx = 0, sumAy = 0, sumAz = 0;
float sumGx = 0, sumGy = 0, sumGz = 0;
int   validSamples = 0;
// ─── PERFORM IMU CALIBRATION ─────────────────────────────────────────────────────
void gyroCalibration() {
  Serial.println();
  Serial.print("[Arduino] "); Serial.println(F("┌──────────────────────────────────────────────────┐"));
  Serial.print("[Arduino] "); Serial.println(F("│           🎯Starting Gyro calibration            │"));
  Serial.print("[Arduino] "); Serial.println(F("└──────────────────────────────────────────────────┘"));
  Serial.println();
  delay(3000);
  // Serial.println();
  // Serial.print("[Arduino] "); Serial.println(F("🎯 Starting IMU calibration..."));
  Serial.print("[Arduino] "); Serial.println(F("Place sensor flat and still. Cal will start in 5 sec..."));
  for (int i = 5; i > 0; i--) {
    Serial.print("[Arduino] "); Serial.print(i); Serial.println(F("..."));
    delay(1000);
  }

  

  Serial.print("[Arduino] "); Serial.print(F("📈 Collecting "));
  Serial.print(numSamples);
  Serial.println(F(" samples:"));

  Serial.print("[Arduino] ");
  for (int i = 0; i < numSamples; i++) {
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
    if ((i & 0x7F) == 0) {
      // print a dot every 128 samples to save serial overhead
      Serial.print(F("."));
    }
    delay(5);  // slightly shorter delay to speed up
  }
  Serial.println();

  if (validSamples > 0) {
    accel_offsetX = sumAx / validSamples;
    accel_offsetY = sumAy / validSamples;
    accel_offsetZ = (sumAz / validSamples) - GRAVITY;  // remove gravity

    gyro_offsetX = sumGx / validSamples;
    gyro_offsetY = sumGy / validSamples;
    gyro_offsetZ = sumGz / validSamples;

    calibrated = true;

    Serial.print("[Arduino] "); Serial.println(F("✅ Calibration complete!"));
    Serial.print("[Arduino] "); Serial.print(F("   Accel offsets (m/s²): "));
    Serial.print(accel_offsetX, 3); Serial.print(F(", "));
    Serial.print(accel_offsetY, 3); Serial.print(F(", "));
    Serial.println(accel_offsetZ, 3);

    Serial.print("[Arduino] "); Serial.print(F("   Gyro offsets (deg/s):  "));
    Serial.print(gyro_offsetX, 3);  Serial.print(F(", "));
    Serial.print(gyro_offsetY, 3);  Serial.print(F(", "));
    Serial.println(gyro_offsetZ, 3);

    Serial.print("[Arduino] "); Serial.println(F("Type 'calibration' anytime to recalibrate."));
  } else {
    Serial.print("[Arduino] "); Serial.println(F("❌ Calibration FAILED. No valid samples collected."));
  }
}
// ─── COMPUTE ROLL, PITCH, YAW ───────────────────────────────────────────────────
void calculateOrientation(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float &roll, float &pitch, float &yaw)
{
  // Roll  = atan2(y, sqrt(x² + z²))
  // Pitch = atan2(-x, sqrt(y² + z²))
  roll  = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / PI;
  pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;

  // Yaw: simple integration of gz (deg/s * dt)
  static float yawInt   = 0;
  static uint32_t lastT = 0;

  uint32_t t = millis();
  if (lastT != 0) {
    float dt = (t - lastT) * 0.001f;  // seconds
    yawInt += gz * dt;
  }
  lastT = t;

  // Wrap into [-180, 180]
  if (yawInt >  180.0f) yawInt -= 360.0f;
  if (yawInt < -180.0f) yawInt += 360.0f;
  yaw = yawInt;
}
// ─── SIMPLE MOTION DETECTION (SQUARED) ─────────────────────────────────────────
void checkMotionSq(float accelMagSq, float gyroMagSq) {
  // Compare |accel| ≈ 9.81 ± threshold, but squared:
  bool accelMotion = (accelMagSq > highSq) || (accelMagSq < lowSq);
  bool gyroMotion  = (gyroMagSq  > gyroThSq);

  if (accelMotion || gyroMotion) {
    if (!motionDetected) {
      // Uncomment if you want a console message on start:
      // Serial.print("[Arduino] "); Serial.println(F("🚶 Motion detected!"));
    }
    motionDetected   = true;
    lastMotionTime   = millis();
  }
  else {
    // If no motion for >2 sec, clear flag
    if (motionDetected && ((millis() - lastMotionTime) > 2000U)) {
      // Uncomment if you want a console message on stop:
      // Serial.print("[Arduino] "); Serial.println(F("🛑 Motion stopped."));
      motionDetected = false;
    }
  }
}
