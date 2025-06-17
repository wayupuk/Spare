#include <Wire.h>
#include <TFT_eSPI.h>  // TFT screen library
#include "Gyro_QMI8658.h"

TFT_eSPI tft = TFT_eSPI();  // Create TFT object

void setup() {
    Serial.begin(115200);
    I2C_Init();      // Initialize I2C communication
    QMI8658_Init();  // Initialize the QMI8658 Gyroscope sensor

    tft.init();
    tft.setRotation(0);  // Start with default rotation
    tft.fillScreen(TFT_BLACK);
}

void loop() {
    getGyroscope();  // Retrieve gyro data
    getAccelerometer(); // Retrieve accelerometer data

    float angleX = Gyro.x;  // Angular velocity around X-axis
    float angleY = Gyro.y;  // Angular velocity around Y-axis
    float accX = Accel.x;   // Accelerometer X value
    float accY = Accel.y;   // Accelerometer Y value

    int rotation = 0;

    // Determine screen orientation based on gyro & accel data
    if (accX > 0.5) {
        rotation = 2;  // Portrait (90°)
    } else if (accX < -0.5) {
        rotation = 0;  // Portrait inverted (270°)
    } else if (accY > 0.5) {
        rotation = 3;  // Landscape (180°)
    } else {
        rotation = 1;  // Landscape inverted (0°)
    }

    tft.setRotation(rotation);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM); // Set text datum to middle centre
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(tft.width() / 2 - 40, tft.height() / 2 - 10);
    tft.print("Hello World");

    Serial.print("Gyro X: "); Serial.print(angleX);
    Serial.print(" | Gyro Y: "); Serial.print(angleY);
    Serial.print(" | Acc X: "); Serial.print(accX);
    Serial.print(" | Acc Y: "); Serial.print(accY);
    Serial.print(" | Rotation: ");Serial.println(rotation);

    delay(50);  // Refresh screen based on new readings
}
