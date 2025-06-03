#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1015 ads1015_1;
Adafruit_ADS1015 ads1015_2;

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Hello!");

  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV)");
  Wire.begin(15, 14);
  ads1015_1.begin(0x48);
  ads1015_2.begin(0x49);
}

void loop(void)
{
  int16_t adc0, adc1, adc2, adc3, adc4;

  adc0 = ads1015_1.readADC_SingleEnded(0);
  adc1 = ads1015_1.readADC_SingleEnded(1);
  adc2 = ads1015_1.readADC_SingleEnded(2);
  adc3 = ads1015_1.readADC_SingleEnded(3);
  adc4 = ads1015_2.readADC_SingleEnded(0);
  Serial.print("AIN1: "); Serial.println(adc4);
  Serial.print("AIN2: "); Serial.println(adc0);
  Serial.print("AIN3: "); Serial.println(adc1);
  Serial.print("AIN4: "); Serial.println(adc2);
  Serial.print("AIN5: "); Serial.println(adc3);
  Serial.println(" ");

  delay(1000);
}