// RECEIVER CODE (e.g., for "warboy")
#include <Arduino.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Adafruit_ADS1X15.h>
#include <QMI8658.h>

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


// ──── PERIPHERAL ADDRESSES & CONSTANTS ─────────────────────────────────────────
const uint8_t ADS1015_1_ADDRESS = 0x48;
const uint8_t ADS1015_2_ADDRESS = 0x49;
const int NUM_FLEX_SENSORS = 5;
const int CALIBRATION_SAMPLES = 1000; // Number of samples for sensor calibration

// ──── PERIPHERAL OBJECTS ───────────────────────────────────────────────────────

Adafruit_ADS1015 ads1015_1;
Adafruit_ADS1015 ads1015_2;
QMI8658          imu;

// ──── CALIBRATION & FILTER VARIABLES ──────────────────────────────────────────
float mean_acd_max[NUM_FLEX_SENSORS] = {0.0f};
float mean_acd_low[NUM_FLEX_SENSORS] = {0.0f};
float low_value = 0.0f;
float high_value = 1000.0f;

// ──── SENSOR VARIABLES ──────────────────────────────────────────
unsigned long start_timestamp = 0; // For CSV time column
float flex_raw_value[NUM_FLEX_SENSORS];
float flex_cal[NUM_FLEX_SENSORS];

// ──── MOTION DETECTION VARIABLES ──────────────────────────────────────────────
static const float GRAVITY         = ONE_G;
static const uint32_t MOTION_STOP_TIMEOUT_MS = 2000; // 2 seconds
// ──── PROTOTYPES ────────────────────────────────────────────────────────────────
void flexCalibration();
void readAllFlexSensors(float* raw_values);
void printCountdown(int seconds, const char* message_prefix = "[Arduino]");
void ADS1015_INITIALIZATION();
void QMI8658_INITIALIZATION();
void checkCommand(){
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read until newline
    command.trim();                                // Remove any trailing CR/LF or spaces
    Serial.print("[Arduino] Received: ");
    Serial.println(command);
    if (command.equalsIgnoreCase("reset")) {
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
    else {
      Serial.print("[Arduino] "); Serial.print("Unknown command: "); Serial.println(command);
    }
  }
}

// ────────────────────────────────────────────────────────────────────────────────────────────
// fc:01:2c:d9:35:0c imortal joe 
// fc:01:2c:d9:3d:90 warboy

// MAC Address of the receiver
uint8_t receiverAddress[] = {0xfc, 0x01, 0x2c, 0xd9, 0x35, 0x0c}; // imortal joe 

String payload;
JsonDocument jsonDoc;

// Define the WiFi channel to use
#define WIFI_CHANNEL 1

// Callback function that is called when data is sent
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status)
{
	Serial.print("\r\nLast Packet Send Status:\t");
	Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Function to register the receiver as a peer
void RegisterPeer()
{
	esp_now_peer_info_t peerInfo;
	memset(&peerInfo, 0, sizeof(peerInfo)); // Initialize peerInfo to all zeros
	memcpy(peerInfo.peer_addr, receiverAddress, 6);
	peerInfo.channel = WIFI_CHANNEL;  // Use the same channel as the sender
	peerInfo.encrypt = false;

	// Add peer
	if (esp_now_add_peer(&peerInfo) != ESP_OK)
	{
		Serial.println("Failed to add peer");
		return;
	}
	Serial.println("Peer registered successfully");
}

// Callback function that is executed when data is received
void OnMessageReceived(const uint8_t* mac_addr, const uint8_t* incomingData, int len)
{
	Serial.printf("Packet received from: %02X:%02X:%02X:%02X:%02X:%02X\n",
	              mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

	JsonDocument doc;
	
    // Deserialize the JSON document directly from the incoming data buffer
	DeserializationError error = deserializeJson(doc, incomingData, len);

	// Check for errors in deserialization
	if (error)
	{
		Serial.print("deserializeJson() failed: ");
		Serial.println(error.c_str());
		return;
	}

	// Extract data from the JSON document
	int packet_id = doc["data"];

    // Print the received data
    String receivedPayload;
    serializeJson(doc, receivedPayload);
	Serial.print("Payload received: ");
    Serial.println(receivedPayload);
	Serial.print("Packet ID: ");
	Serial.println(packet_id);
    Serial.println("---------------------------------");
}

// Function to initialize ESP-NOW
void InitEspNow()
{
	if (esp_now_init() != ESP_OK)
	{
		Serial.println("Error initializing ESP-NOW");
		return;
	}
	
	// Register the receive callback
	esp_now_register_recv_cb(OnMessageReceived);
	esp_now_register_send_cb(OnDataSent);
	RegisterPeer();

  // Serial.println("ESP-NOW Initialized. Waiting for messages...");
}

void SendMessage()
{
	
	serializeJson(jsonDoc, payload);

	esp_err_t result = esp_now_send(receiverAddress, (uint8_t*)payload.c_str(), payload.length());

	if (result == ESP_OK)
	{
		Serial.println("Sent with success");
	}
	else
	{
		Serial.println("Error sending the data");
	}
}

void setup()
{
	Serial.begin(115200);
	Serial.println("Receiver ESP32 starting...");
    
    // 1. Set device as a Wi-Fi Station
	WiFi.mode(WIFI_AP_STA);

  // 2. Set the WiFi Channel to match the sender
  // This is the most important step!
  if (esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
      Serial.println("Error setting WiFi channel");
      return;
  }
  
  // ─── ADS1015 INITIALIZATION ─────────────────────────────────────────────────
  ADS1015_INITIALIZATION();//a 12-bit analog-to-digital converter (ADC) that operates over the I2C protocol

  // ─── QMI8658 INITIALIZATION ─────────────────────────────────────────────────
  QMI8658_INITIALIZATION();

  // 3. Initialize ESP-NOW
	InitEspNow();
}

void loop()
{
  checkCommand();
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
  
  // jsonDoc["data_int"] = 123456789;
  // jsonDoc["data_float"] = 12.3456789;
  jsonDoc["flex_0"] = flex_cal[0];
  jsonDoc["flex_1"] = flex_cal[1];
  jsonDoc["flex_2"] = flex_cal[2];
  jsonDoc["flex_3"] = flex_cal[3];
  jsonDoc["flex_4"] = flex_cal[4];

	SendMessage();
  delay(100);
	// The loop can be empty as all the work is done in the callback function.
}

void readAllFlexSensors(float* raw_values) {
    raw_values[0] = ads1015_2.readADC_SingleEnded(0);
    raw_values[1] = ads1015_1.readADC_SingleEnded(0);
    raw_values[2] = ads1015_1.readADC_SingleEnded(1);
    raw_values[3] = ads1015_1.readADC_SingleEnded(2);
    raw_values[4] = ads1015_1.readADC_SingleEnded(3);
}

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
void printCountdown(int seconds, const char* message_prefix) {
    for (int i = seconds; i > 0; i--) {
        Serial.printf("%s %d...\n", message_prefix, i);
        delay(1000);
    }
}

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