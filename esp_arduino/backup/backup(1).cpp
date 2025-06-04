#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <QMI8658.h>

Adafruit_ADS1015 ads1015_1;
Adafruit_ADS1015 ads1015_2;
QMI8658 imu;
QMI8658_Data sensorData;

void setup(void)
{
    Serial.begin(115200);
    delay(1000); // Wait for serial to stabilize
    Serial.println("Getting single-ended readings from AIN0..3");
    Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV)");
    Wire.begin(15, 14);
    ads1015_1.begin(0x48);
    ads1015_2.begin(0x49);


    Serial.println("🚀 QMI8658 Enhanced Library - Flexible Units & Precision");
    Serial.println("========================================================");
      
    // Initialize the sensor with pins 6,7 (uses Wire1 automatically on RP2040)
    Serial.println("📍 Initializing sensor...");
    bool success = imu.begin(6, 7);
    
    if (!success) {
        Serial.println("❌ Failed to initialize QMI8658!");
        Serial.println("Please check:");
        Serial.println("- Wiring connections (SDA=6, SCL=7)");
        Serial.println("- Power supply (3.3V)");
        Serial.println("- I2C address");
        while (1) {
            Serial.println("⏳ Retrying in 5 seconds...");
            delay(5000);
        }
    }
    
    Serial.println("✅ QMI8658 initialized successfully!");
    Serial.print("WHO_AM_I: 0x");
    Serial.println(imu.getWhoAmI(), HEX);
    
    // Configure sensor settings
    Serial.println("\n⚙️ Configuring sensor...");
    
    // Set accelerometer range (±8g)
    imu.setAccelRange(QMI8658_ACCEL_RANGE_8G);
    
    // Set accelerometer output data rate (1000Hz)
    imu.setAccelODR(QMI8658_ACCEL_ODR_1000HZ);
    
    // Set gyroscope range (±512dps)
    imu.setGyroRange(QMI8658_GYRO_RANGE_512DPS);
    
    // Set gyroscope output data rate (1000Hz)
    imu.setGyroODR(QMI8658_GYRO_ODR_1000HZ);
    
    // ⭐ NEW: Configure units and precision
    Serial.println("\n🎯 Setting units and precision...");
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set units (DEFAULT: mg for accel, dps for gyro - matches most IMU displays)
    // imu.setAccelUnit_mg(true);      // Use mg (like your screen: ACC_X = -965.82)
    // imu.setGyroUnit_dps(true);      // Use dps (degrees per second)
    // imu.setDisplayPrecision(6);     // 6 decimal places (like your screen)
    
    // Alternative ways to set units:
    imu.setAccelUnit_mps2(true);  // Would use m/s² instead of mg
    imu.setGyroUnit_rads(true);   // Would use rad/s instead of dps
    imu.setDisplayPrecision(QMI8658_PRECISION_4); // 4 decimal places
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // Enable sensors
    imu.enableSensors(QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);
    
    Serial.println("✅ Configuration complete!");
    
    // Show current settings
    Serial.println("\n📋 Current Settings:");
    Serial.print("   Accelerometer unit: ");
    Serial.println(imu.isAccelUnit_mg() ? "mg" : "m/s²");
    Serial.print("   Gyroscope unit: ");
    Serial.println(imu.isGyroUnit_dps() ? "dps" : "rad/s");
    Serial.print("   Display precision: ");
    Serial.print(imu.getDisplayPrecision());
    Serial.println(" decimal places");
    
    Serial.println("\n📊 Starting sensor readings...");
    Serial.println("Time(ms)\tAcc_X(mg)\tAcc_Y(mg)\tAcc_Z(mg)\tGyro_X(dps)\tGyro_Y(dps)\tGyro_Z(dps)\tTemp(°C)");
    Serial.println("-----------------------------------------------------------------------------------------");
    
    delay(100); // Allow sensor to stabilize

}

void loop(void)
{
  int16_t adc0, adc1, adc2, adc3, adc4;
  float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
  if (imu.readSensorData(sensorData)) {
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // flex sensor
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // นิ้วโป้งหรรสา
      adc0 = ads1015_2.readADC_SingleEnded(0);
      adc1 = ads1015_1.readADC_SingleEnded(0);
      adc2 = ads1015_1.readADC_SingleEnded(1);
      adc3 = ads1015_1.readADC_SingleEnded(2);
      adc4 = ads1015_1.readADC_SingleEnded(3);
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // accel and gyro sensor
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // accel //
      accel_x = sensorData.accelX;
      accel_y = sensorData.accelY;
      accel_z = sensorData.accelZ;
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // gyro //
      gyro_x = sensorData.gyroX;
      gyro_y = sensorData.gyroY;
      gyro_z = sensorData.gyroZ;
  
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      Serial.print(adc0); Serial.print(",");
      Serial.print(adc1); Serial.print(",");
      Serial.print(adc2); Serial.print(",");
      Serial.print(adc3); Serial.print(",");
      Serial.print(adc4); Serial.print(",");
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      Serial.print(accel_x, 4); Serial.print(",");
      Serial.print(accel_y, 4); Serial.print(",");
      Serial.print(accel_z, 4); Serial.print(",");
      // Serial.print(sensorData.accelX, 6); Serial.print(",");
      // Serial.print(sensorData.accelY, 6); Serial.print(",");
      // Serial.print(sensorData.accelZ, 6); Serial.print(",");
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      Serial.print(gyro_x, 4); Serial.print(",");
      Serial.print(gyro_y, 4); Serial.print(",");
      Serial.print(gyro_z, 4); Serial.print("\n");
      // Serial.print(sensorData.gyroX, 6); Serial.print(",");
      // Serial.print(sensorData.gyroY, 6); Serial.print(",");
      // Serial.print(sensorData.gyroZ, 6); Serial.print("\n");
  } else {
      Serial.println("❌ Failed to read sensor data!");
  }
  // Serial.println(adc4);
  

  
  // if (imu.readSensorData(sensorData)) {
  //     // Serial.print(millis());
  //     Serial.print("\t");
  //     Serial.print(sensorData.accelX, 6);
  //     Serial.print("\t");
  //     Serial.print(sensorData.accelY, 6);
  //     Serial.print("\t");
  //     Serial.print(sensorData.accelZ, 6);
  //     Serial.print("\t");
  //     Serial.print(sensorData.gyroX, 6);
  //     Serial.print("\t");
  //     Serial.print(sensorData.gyroY, 6);
  //     Serial.print("\t");
  //     Serial.print(sensorData.gyroZ, 6);
  //     Serial.print("\t");
  //     Serial.println(sensorData.temperature, 1);
  // } else {
  //     Serial.println("❌ Failed to read sensor data!");
  // }
  // Serial.print("AIN1: "); Serial.println(adc4);
  // Serial.print("AIN2: "); Serial.println(adc0);
  // Serial.print("AIN3: "); Serial.println(adc1);
  // Serial.print("AIN4: "); Serial.println(adc2);
  // Serial.print("AIN5: "); Serial.println(adc3);
  // Serial.println(" ");

  delay(100);
}