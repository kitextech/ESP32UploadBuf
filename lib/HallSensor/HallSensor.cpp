/*
RPM part is based on Arduino Hall Effect Sensor Project by Arvind Sanjeev
Link: http://diyhacking.com
Temperature part is based on Adafruit Learning System guide on Thermistors by Limor Fried, Adafruit Industries
Link:  https://learn.adafruit.com/thermistor/overview

Both modified by Bertalan Kovács (bertalan@kitex.tech)
Install ESP8266 on Arduino IDE: https://github.com/esp8266/Arduino/blob/master/README.md
*/

#include "HallSensor.h"

HallSensor::HallSensor(uint16_t uploadFreq)
{
  uploadFrequency = uploadFreq;
  t0 = millis();
  Serial.println("Successfully created a hall sensor handling object");
}

void HallSensor::setup()
{
  pinMode(PIN_HALL, INPUT_PULLUP);                                         // Pulling up the pin (equivalent of using the 10kOhm resistance on the board)
  attachInterrupt(digitalPinToInterrupt(HALL), magnet_detect, RISING); // Initialize the intterrupt pin
}

Speed HallSensor::prepareData(int64_t time)
{
  Speed HallData = Speed_init_zero;
  HallData.time = time;

  return HallData;
}