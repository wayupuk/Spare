#include <ArduinoBLE.h>

// --- SensorData struct definition (from CommonData.h content) ---
#ifndef COMMON_DATA_H
#define COMMON_DATA_H
#include <stdint.h>
struct SensorData {
  uint32_t timestampMs; uint8_t motionDetected; float roll; float pitch; float yaw;
  float filteredAccelX; float filteredAccelY; float filteredAccelZ;
  float filteredGyroX; float filteredGyroY; float filteredGyroZ;
  float accelMagSq; float gyroMagSq; float flex_cal[5];
};
#endif // COMMON_DATA_H
// --- End of SensorData struct definition ---

SensorData receivedVal; // Instance to store the received data. "val"

// UUIDs must match the Peripheral's definitions
const char* serviceUUID = "19B10000-E8F2-537E-4F6C-D104768A1214";
const char* characteristicUUID = "19B10003-E8F2-537E-4F6C-D104768A1214"; // Updated UUID for raw data

BLEDevice peripheral;

// It's good practice to declare the buffer globally or ensure it's in scope when used.
// However, for readValue, a local buffer in the loop is also fine if used immediately.
// uint8_t byteBuffer[sizeof(SensorData)]; // Can be global, or local as shown in loop

void setup() {
  Serial.begin(9600);
  // while (!Serial && millis() < 5000); // Optional: Wait for serial, timeout after 5s

  Serial.println("Starting BLE Central - Raw Sensor Data Receiver");
  Serial.print("Expected size of SensorData struct: ");
  Serial.println(sizeof(SensorData));

  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1); // Halt
  }

  Serial.println("Scanning for BLE Peripherals...");
  // Scan for the specific service UUID for efficiency
  BLE.scanForUuid(serviceUUID);
  // Or, if you know the peripheral's name:
  // BLE.scanForName("RawSensorDataProvider");
}

void loop() {
  peripheral = BLE.available(); // Check if a peripheral was discovered

  if (peripheral) {
    Serial.println("* Peripheral found!");
    Serial.print("  Device Name: "); Serial.println(peripheral.localName());
    Serial.print("  Advertised Service: "); Serial.println(peripheral.advertisedServiceUuid());

    BLE.stopScan(); // Stop scanning once we've found a device

    Serial.print("Connecting to '"); Serial.print(peripheral.localName()); Serial.print("' ...");
    if (peripheral.connect()) {
      Serial.println(" Connected.");

      Serial.println("Discovering attributes...");
      if (peripheral.discoverAttributes()) {
        Serial.println("Attributes discovered.");

        BLEService customService = peripheral.service(serviceUUID);
        if (customService) {
          Serial.print("Found Service: "); Serial.println(customService.uuid());
          BLECharacteristic dataChar = customService.characteristic(characteristicUUID);

          if (dataChar) {
            Serial.print("Found Characteristic: "); Serial.println(dataChar.uuid());
            Serial.print("  Max Value Length: "); Serial.println(dataChar.valueSize()); // Max size it can hold
            Serial.print("  Properties: "); Serial.println(dataChar.properties(), HEX);

            if (dataChar.canSubscribe() && dataChar.properties() & BLENotify) {
                 Serial.println("Subscribing to characteristic notifications...");
                 if (dataChar.subscribe()) {
                    Serial.println("Subscription successful!");
                 } else {
                    Serial.println("Subscription failed!");
                 }
            } else if (!dataChar.canRead()){
                 Serial.println("Characteristic cannot be read or subscribed to.");
            }


            // Main loop while connected
            while (peripheral.connected()) {
              // Check if the characteristic value has been updated (for notifications)
              // Or, if not using notifications and just polling, this check always proceeds to read.
              if (dataChar.valueUpdated() || !(dataChar.properties() & BLENotify) ) { // if value updated or if not using notify (polling)

                if (dataChar.valueLength() == sizeof(SensorData)) {
                  uint8_t localByteBuffer[sizeof(SensorData)]; // Local buffer for the read
                  int bytesRead = dataChar.readValue(localByteBuffer, sizeof(SensorData));

                  if (bytesRead == sizeof(SensorData)) {
                    memcpy(&receivedVal, localByteBuffer, sizeof(SensorData));

                    // Now use receivedVal
                    Serial.println("--- Received Data ---");
                    Serial.print("Timestamp: "); Serial.println(receivedVal.timestampMs);
                    Serial.print("Motion: ");    Serial.println(receivedVal.motionDetected);
                    Serial.print("Roll: ");      Serial.println(receivedVal.roll, 2); // Print float with 2 decimal places
                    Serial.print("Pitch: ");     Serial.println(receivedVal.pitch, 2);
                    Serial.print("Yaw: ");       Serial.println(receivedVal.yaw, 2);
                    Serial.print("AccelX: ");    Serial.println(receivedVal.filteredAccelX, 4);
                    Serial.print("AccelY: ");    Serial.println(receivedVal.filteredAccelY, 4);
                    Serial.print("AccelZ: ");    Serial.println(receivedVal.filteredAccelZ, 4);
                    Serial.print("GyroX: ");     Serial.println(receivedVal.filteredGyroX, 4);
                    Serial.print("GyroY: ");     Serial.println(receivedVal.filteredGyroY, 4);
                    Serial.print("GyroZ: ");     Serial.println(receivedVal.filteredGyroZ, 4);
                    Serial.print("AccelMagSq: ");Serial.println(receivedVal.accelMagSq, 4);
                    Serial.print("GyroMagSq: "); Serial.println(receivedVal.gyroMagSq, 4);
                    for (int i = 0; i < 5; i++) {
                      Serial.print("Flex["); Serial.print(i); Serial.print("]: ");
                      Serial.println(receivedVal.flex_cal[i], 2);
                    }
                    Serial.println("---------------------");
                  } else if (bytesRead > 0) {
                     Serial.print("Error: Read "); Serial.print(bytesRead); Serial.print(" bytes, expected "); Serial.println(sizeof(SensorData));
                  } // else, bytesRead == 0, likely no new data if polling a non-notifying characteristic without change
                } else if (dataChar.valueLength() > 0) { // Length doesn't match
                     Serial.print("Characteristic has data ("); Serial.print(dataChar.valueLength());
                     Serial.print(" bytes), but length does not match expected struct size (");
                     Serial.print(sizeof(SensorData)); Serial.println(" bytes). Waiting for update...");
                }
              } // end valueUpdated or polling check
              delay(100); // Small delay to allow other BLE tasks, adjust as needed
            } // end while(peripheral.connected())
            Serial.println("Peripheral disconnected.");
          } else { // No such characteristic
            Serial.print("Characteristic not found: "); Serial.println(characteristicUUID);
          }
        } else { // No such service
          Serial.print("Service not found: "); Serial.println(serviceUUID);
        }
      } else { // Attribute discovery failed
        Serial.println("Attribute discovery failed!");
      }
      // peripheral.disconnect(); // Already handled by while(peripheral.connected()) loop ending
    } else { // Connection failed
      Serial.println(" Failed to connect.");
    }

    // Restart scanning after disconnection or failed connection attempt
    Serial.println("\nRestarting scan...");
    BLE.scanForUuid(serviceUUID);
  } // end if(peripheral)
  // delay(100); // Small delay in the main loop if no peripheral found yet
}