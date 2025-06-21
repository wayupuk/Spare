#include <ArduinoBLE.h>

// --- SensorData struct definition (from CommonData.h content) ---
#ifndef COMMON_DATA_H
#define COMMON_DATA_H
#include <stdint.h>
struct SensorData {
  float gx, gy, gz;
  float ax, ay, az;
  float angle_x, angle_y, angle_z;
  float flex_cal[5];
};
#endif // COMMON_DATA_H
// --- End of SensorData struct definition ---

SensorData mySensorData; // Create an instance of the struct
unsigned long start_timestamp = 0;

// Define a custom service and characteristic UUID
BLEService customService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic dataCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214", // UUID for raw data
                                     BLERead | BLENotify, sizeof(SensorData));

BLEDevice central;

const char* deviceName = "RawSensorDataProvider";

void setup() {
  Serial.begin(9600);
  // while (!Serial && millis() < 5000); // Optional: Wait for serial, timeout after 5s

  Serial.println("Starting BLE Peripheral - Raw Sensor Data Provider");
  Serial.print("Size of SensorData struct: ");
  Serial.println(sizeof(SensorData));

  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1); // Halt
  }

  BLE.setLocalName(deviceName);
  BLE.setAdvertisedService(customService);
  customService.addCharacteristic(dataCharacteristic);
  BLE.addService(customService);

  // Initialize struct with default values (optional, as it's populated in loop)
  memset(&mySensorData, 0, sizeof(SensorData));
  // Set an initial value for the characteristic (optional)
  // dataCharacteristic.writeValue((uint8_t*)&mySensorData, sizeof(SensorData));


  BLE.advertise();
  Serial.println("Bluetooth device is now advertising");
  Serial.print("Connect to device named: "); Serial.println(deviceName);
  Serial.print("Service UUID: "); Serial.println(customService.uuid());
  Serial.print("Characteristic UUID: "); Serial.println(dataCharacteristic.uuid());

  start_timestamp = millis();
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    if (!central.connected()) { // Check if we just lost connection
        Serial.print("Disconnected from central: ");
        Serial.println(central.address());
    }
    
    // If a central is connected to the peripheral:
    while (central.connected()) {
      // --- Populate the struct with actual or simulated sensor data ---
      mySensorData.timestampMs = millis() - start_timestamp;
      
      // Simulate motion toggling
      if ((millis() / 1000) % 2 == 0) { // Toggle every second for demo
          mySensorData.motionDetected = 1;
      } else {
          mySensorData.motionDetected = 0;
      }

      mySensorData.roll = 10.0f + (sin(millis() / 1000.0f) * 5.0f); // Example: varying roll
      mySensorData.pitch = 5.0f + (cos(millis() / 1500.0f) * 2.0f); // Example: varying pitch
      mySensorData.yaw += 0.5f; if (mySensorData.yaw > 180.0f) mySensorData.yaw = -180.0f;
      
      mySensorData.filteredAccelX = 0.1f * sin(millis()/500.0f);
      mySensorData.filteredAccelY = 0.1f * cos(millis()/500.0f);
      mySensorData.filteredAccelZ = -9.8f + 0.05f * sin(millis()/2000.0f);
      
      mySensorData.filteredGyroX  = 0.2f * sin(millis()/700.0f);
      mySensorData.filteredGyroY  = 0.2f * cos(millis()/700.0f);
      mySensorData.filteredGyroZ  = 0.05f * sin(millis()/1200.0f);
      
      mySensorData.accelMagSq     = sq(mySensorData.filteredAccelX) + sq(mySensorData.filteredAccelY) + sq(mySensorData.filteredAccelZ);
      mySensorData.gyroMagSq      = sq(mySensorData.filteredGyroX) + sq(mySensorData.filteredGyroY) + sq(mySensorData.filteredGyroZ);

      for (int i = 0; i < 5; i++) {
        mySensorData.flex_cal[i] = 100.0f + (i * 50.0f) + 20.0f * sin((millis() / 1000.0f) + i);
      }
      // --- End of data population ---

      // Update the characteristic value with the raw bytes of the struct
      // This will also send a notification if the central has subscribed
      dataCharacteristic.writeValue((uint8_t*)&mySensorData, sizeof(SensorData));

      // Optional: Print some values for debugging on the sender side
      // Serial.print("Sent Timestamp: "); Serial.println(mySensorData.timestampMs);
      // Serial.print("Sent Roll: "); Serial.println(mySensorData.roll);

      delay(500); // Send data every 0.5 seconds (adjust as needed)
    }
    // If the while loop breaks, it means the central disconnected
    Serial.print("Central disconnected: ");
    Serial.println(central.address());
  }
}