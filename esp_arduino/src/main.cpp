#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <time.h>
#include <esp_sntp.h>

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

// ──── WIFI & NTP SETTINGS ───────────────────────────────────────────────────────
const char *ssid       = "LoveSugarRain_2.4G";
const char *password   = "0954849012";

const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec   = 3600;      // GMT+1 (if you prefer fixed offsets; but we use TZ string below)
const int   daylightOffset_sec = 3600;   // DST offset (unused if TZ string handles it)

const char *timeZone = "CST-8";  // Thailand: UTC+7 (TZ string format for ESP32)

// ──── PERIPHERAL OBJECTS ───────────────────────────────────────────────────────
SensorPCF8563    rtc;
Adafruit_ADS1015 ads1015_1;
Adafruit_ADS1015 ads1015_2;
QMI8658          imu;

// ──── CALIBRATION & FILTER VARIABLES ──────────────────────────────────────────
float accel_offsetX = 0, accel_offsetY = 0, accel_offsetZ = 0;
float gyro_offsetX  = 0, gyro_offsetY  = 0, gyro_offsetZ  = 0;
bool  calibrated    = false;

// Simple low-pass filter state
const float alpha = 0.1f;
float filteredAccelX = 0, filteredAccelY = 0, filteredAccelZ = 0;
float filteredGyroX  = 0, filteredGyroY  = 0, filteredGyroZ  = 0;

// ──── MOTION DETECTION VARIABLES ──────────────────────────────────────────────
const float accelThreshold = 2.0f;   // m/s² deviation from 9.81 m/s²
const float gyroThreshold  = 10.0f;  // deg/s
bool  motionDetected       = false;
unsigned long lastMotionTime = 0;

// ──── PROTOTYPES ────────────────────────────────────────────────────────────────
// void timeSyncCallback(struct timeval *tv);
void performCalibration();
void calculateOrientation(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float &roll, float &pitch, float &yaw);
void checkMotion(float accelMag, float gyroMag);

void setup() {
  // ─── SERIAL & WIRE ───────────────────────────────────────────────────────────
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println();
  Serial.println("┌──────────────────────────────────────────────────┐");
  Serial.println("│      ESP32 Sensor Logging (ADS1015 + QMI8658)    │");
  Serial.println("└──────────────────────────────────────────────────┘");
  Serial.println();

  // ─── RTC (PCF8563) INITIALIZATION ────────────────────────────────────────────
  pinMode(SENSOR_IRQ, INPUT_PULLUP);
  Wire.begin(SENSOR_SDA, SENSOR_SCL);
  if (!rtc.begin(Wire, SENSOR_SDA, SENSOR_SCL)) {
    Serial.println("❌ Failed to find PCF8563 RTC. Check wiring!");
    while (true) {
      delay(1000);
      Serial.println("⏳ Waiting for PCF8563...");
    }
  }
  Serial.println("✅ PCF8563 RTC initialized.");

  // ─── SNTP / NTP-TO-RTC SYNCHRO ────────────────────────────────────────────────
  // sntp_set_time_sync_notification_cb(timeSyncCallback);
  // configTzTime(timeZone, ntpServer1, ntpServer2);
  // Serial.println("🔄 SNTP client started, waiting for time sync...");

  // ─── WI-FI SETUP ─────────────────────────────────────────────────────────────
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - wifiStart > 10000) {
      Serial.println();
      Serial.println("⚠️  Wi-Fi connect timeout. Retrying...");
      wifiStart = millis();
      WiFi.begin(ssid, password);
    }
  }
  Serial.println();
  Serial.print("✅ Connected to Wi-Fi. IP: ");
  Serial.println(WiFi.localIP());

  // ─── ADS1015 INITIALIZATION ─────────────────────────────────────────────────
  Serial.println();
  Serial.println("Initializing ADS1015 #1 (address 0x48)...");
  if (!ads1015_1.begin(0x48)) {
    Serial.println("❌ Failed to find ADS1015 at 0x48. Check wiring/address!");
    while (true) {
      delay(1000);
      Serial.println("⏳ Waiting for ADS1015 #1...");
    }
  }
  Serial.println("✅ ADS1015 #1 OK.");

  Serial.println("Initializing ADS1015 #2 (address 0x49)...");
  if (!ads1015_2.begin(0x49)) {
    Serial.println("❌ Failed to find ADS1015 at 0x49. Check wiring/address!");
    while (true) {
      delay(1000);
      Serial.println("⏳ Waiting for ADS1015 #2...");
    }
  }
  Serial.println("✅ ADS1015 #2 OK.");

  // ─── QMI8658 IMU INITIALIZATION ───────────────────────────────────────────────
  Serial.println();
  Serial.println("Initializing QMI8658 IMU...");
  if (!imu.begin(SENSOR_SDA, SENSOR_SCL)) {
    Serial.println("❌ Failed to initialize QMI8658. Check wiring!");
    while (true) {
      delay(1000);
      Serial.println("⏳ Waiting for QMI8658...");
    }
  }
  Serial.println("✅ QMI8658 initialized.");
  Serial.print("   WHO_AM_I: 0x");
  Serial.println(imu.getWhoAmI(), HEX);

  // Configure ranges, ODRs, units
  imu.setAccelRange(QMI8658_ACCEL_RANGE_8G);
  imu.setAccelODR(QMI8658_ACCEL_ODR_1000HZ);
  imu.setGyroRange(QMI8658_GYRO_RANGE_512DPS);
  imu.setGyroODR(QMI8658_GYRO_ODR_1000HZ);
  imu.setAccelUnit_mps2(true);  // output in m/s²
  imu.setGyroUnit_rads(false);  // output in deg/s
  imu.enableSensors(QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);
  Serial.println("🔧 QMI8658 sensor configured (8g/512dps, 1000Hz).");

  // ─── CALIBRATION ───────────────────────────────────────────────────────────────
  performCalibration();

  // ─── HEADER FOR CSV OUTPUT ────────────────────────────────────────────────────
  Serial.println();
  Serial.println("Time(ms),Motion,Roll,Pitch,Yaw,Accel_Mag,Gyro_Mag,ADC0,ADC1,ADC2,ADC3,ADC4");
  Serial.println("-------------------------------------------------------------------------------");
}

void loop() {
  // ─── 1) GET & PRINT LOCAL TIME ───────────────────────────────────────────────
  // struct tm timeinfo;
  // if (getLocalTime(&timeinfo)) {
  //   char timeBuf[64];
  //   strftime(timeBuf, sizeof(timeBuf), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  //   Serial.println(timeBuf);
  // } else {
  //   // Silently skip if SNTP hasn’t synced yet (or if it failed briefly)
  //   // Serial.println("⚠️  Failed to obtain local time.");
  // }

  // ─── 2) READ ADS1015 ADC CHANNELS ───────────────────────────────────────────
  int16_t adc0 = ads1015_2.readADC_SingleEnded(0);
  int16_t adc1 = ads1015_1.readADC_SingleEnded(0);
  int16_t adc2 = ads1015_1.readADC_SingleEnded(1);
  int16_t adc3 = ads1015_1.readADC_SingleEnded(2);
  int16_t adc4 = ads1015_1.readADC_SingleEnded(3);

  // ─── 3) READ & PROCESS IMU ───────────────────────────────────────────────────
  QMI8658_Data data;
  float roll=0, pitch=0, yaw=0;
  float accelMagnitude = 0, gyroMagnitude = 0;

  bool gotIMU = imu.readSensorData(data);
  if (gotIMU) {
    // Apply calibration offsets
    float accelX = data.accelX - accel_offsetX;
    float accelY = data.accelY - accel_offsetY;
    float accelZ = data.accelZ - accel_offsetZ;

    float gyroX  = data.gyroX  - gyro_offsetX;
    float gyroY  = data.gyroY  - gyro_offsetY;
    float gyroZ  = data.gyroZ  - gyro_offsetZ;

    // Low-pass filter
    filteredAccelX = alpha * accelX + (1.0f - alpha) * filteredAccelX;
    filteredAccelY = alpha * accelY + (1.0f - alpha) * filteredAccelY;
    filteredAccelZ = alpha * accelZ + (1.0f - alpha) * filteredAccelZ;

    filteredGyroX  = alpha * gyroX  + (1.0f - alpha) * filteredGyroX;
    filteredGyroY  = alpha * gyroY  + (1.0f - alpha) * filteredGyroY;
    filteredGyroZ  = alpha * gyroZ  + (1.0f - alpha) * filteredGyroZ;

    // Compute orientation
    calculateOrientation(
        filteredAccelX, filteredAccelY, filteredAccelZ,
        filteredGyroX,  filteredGyroY,  filteredGyroZ,
        roll, pitch, yaw
    );

    // Magnitudes for motion detection
    accelMagnitude = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
    gyroMagnitude  = sqrt(gyroX*gyroX   + gyroY*gyroY   + gyroZ*gyroZ);

    checkMotion(accelMagnitude, gyroMagnitude);
  } else {
    // If IMU read fails, keep previous filtered values and motion state
    // (you could print a warning if desired)
  }

  // ─── 4) CHECK FOR “cal” COMMAND TO RE-CALIBRATE ───────────────────────────────
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equalsIgnoreCase("cal") || command.equalsIgnoreCase("calibrate")) {
      Serial.println();
      Serial.println("🔄 Re-calibrating IMU...");
      performCalibration();
    }
  }

  // ─── 5) PRINT A CSV LINE: TimeSinceStart,Motion,Roll,Pitch,Yaw,AccelMag,GyroMag,ADC0..ADC4 ───
  unsigned long nowMs = millis();
  Serial.print(nowMs); Serial.print(",");
  Serial.print(motionDetected); Serial.print(",");
  
  Serial.print(roll,  6); Serial.print(",");
  Serial.print(pitch, 6); Serial.print(",");
  Serial.print(yaw,   6); Serial.print(",");

  Serial.print(filteredAccelX, 6); Serial.print(",");
  Serial.print(filteredAccelY, 6); Serial.print(",");
  Serial.print(filteredAccelZ, 6); Serial.print(",");
  Serial.print(filteredGyroX, 6); Serial.print(",");
  Serial.print(filteredGyroY, 6); Serial.print(",");
  Serial.print(filteredGyroZ, 6); Serial.print(",");

  Serial.print(accelMagnitude, 6); Serial.print(",");
  Serial.print(gyroMagnitude,  6); Serial.print(",");

  Serial.print(adc0); Serial.print(",");
  Serial.print(adc1); Serial.print(",");
  Serial.print(adc2); Serial.print(",");
  Serial.print(adc3); Serial.print(",");
  Serial.print(adc4);
  Serial.println();

  delay(10);  // 10 Hz loop
}

// ─── CALLBACK: WHEN SNTP SYNC FINISHES, UPDATE RTC ─────────────────────────────
// void timeSyncCallback(struct timeval *tv) {
//   Serial.println("⏰ Time synced via NTP. Writing to PCF8563 RTC...");
//   rtc.hwClockWrite();
// }

// ─── PERFORM IMU CALIBRATION ─────────────────────────────────────────────────────
void performCalibration() {
  Serial.println();
  Serial.println("🎯 Starting IMU calibration...");
  Serial.println("Place sensor flat and still. Cal will start in 3 sec...");
  for (int i = 3; i > 0; i--) {
    Serial.print(i);
    Serial.println("...");
    delay(1000);
  }

  const int numSamples = 1000;
  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  int   validSamples = 0;

  Serial.print("📈 Collecting ");
  Serial.print(numSamples);
  Serial.println(" samples:");

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
    if (i % 100 == 0) {
      Serial.print(".");
    }
    delay(10);
  }
  Serial.println();

  if (validSamples > 0) {
    accel_offsetX = sumAx / validSamples;
    accel_offsetY = sumAy / validSamples;
    accel_offsetZ = (sumAz / validSamples) - 9.81f;  // remove gravity

    gyro_offsetX = sumGx / validSamples;
    gyro_offsetY = sumGy / validSamples;
    gyro_offsetZ = sumGz / validSamples;

    calibrated = true;

    Serial.println("✅ Calibration complete!");
    Serial.print("   Accel offsets (m/s²): ");
    Serial.print(accel_offsetX, 3); Serial.print(", ");
    Serial.print(accel_offsetY, 3); Serial.print(", ");
    Serial.println(accel_offsetZ, 3);

    Serial.print("   Gyro offsets (deg/s):  ");
    Serial.print(gyro_offsetX, 3);  Serial.print(", ");
    Serial.print(gyro_offsetY, 3);  Serial.print(", ");
    Serial.println(gyro_offsetZ, 3);

    Serial.println("Type 'cal' anytime to recalibrate.");
  } else {
    Serial.println("❌ Calibration FAILED. No valid samples collected.");
  }
}

// ─── COMPUTE ROLL, PITCH, YAW ───────────────────────────────────────────────────
void calculateOrientation(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float &roll, float &pitch, float &yaw)
{
  // Roll  = atan2(y, sqrt(x² + z²))
  // Pitch = atan2(-x, sqrt(y² + z²))
  roll  = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / M_PI;
  pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;

  // Simple yaw integration (deg/s * dt)
  static float yawInt   = 0;
  static unsigned long lastTime = 0;

  unsigned long t = millis();
  if (lastTime > 0) {
    float dt = (t - lastTime) / 1000.0f;  // in seconds
    yawInt += gz * dt;                   // gz is already in deg/s
  }
  lastTime = t;

  // Keep yaw in [-180, +180]
  yaw = yawInt;
  while (yaw >  180.0f) yaw -= 360.0f;
  while (yaw < -180.0f) yaw += 360.0f;
}

// ─── SIMPLE MOTION DETECTION ───────────────────────────────────────────────────
void checkMotion(float accelMag, float gyroMag) {
  // Compare |accel| to 9.81 ± threshold, or check gyro rate
  float accelDev = fabsf(accelMag - 9.81f);
  if (accelDev > accelThreshold || gyroMag > gyroThreshold) {
    if (!motionDetected) {
      // Serial.println("🚶 Motion detected!");
    }
    motionDetected   = true;
    lastMotionTime   = millis();
  } else {
    // If no motion for >2 sec, reset flag
    if (motionDetected && (millis() - lastMotionTime > 2000)) {
      // Serial.println("🛑 Motion stopped.");
      motionDetected = false;
    }
  }
}
