/*
Based on Arduino Hall Effect Sensor Project by Arvind Sanjeev
Link: http://diyhacking.com

Modified by Bertalan Kovács (bertalan@kitex.tech) and Mathias Neuenschwander (mathias@kitex.tech)
*/

#include "HallSensor.h"

HallSensor::HallSensor(uint8_t pinHall, uint8_t polePairNum, uint16_t uploadFreq)
{
  PIN_HALL = pinHall;
  POLE_PAIR_NUM = polePairNum;
  uploadFrequency = uploadFreq;
  t0 = millis();
  Serial.println("Successfully created a hall sensor handling object");
}

void HallSensor::setup()
{
  // detection = 0;
  pinMode(PIN_HALL, INPUT_PULLUP);                                         // Pulling up the pin (equivalent of using the 10kOhm resistance on the board)
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), magnet_detect, RISING); // Initialize the intterrupt pin
}

ICACHE_RAM_ATTR void HallSensor::magnet_detect() // This function is called whenever a magnet/interrupt is detected
{
  detection++;
}

Speed HallSensor::prepareData(int64_t time)
{
  Speed rpmData = Speed_init_zero;
  rpmData.time = time;

  // 60 is to convert rps to rpm; 1000 to corrigate ms to s;
  // division by pole pairs is to get the whole rotation not only between two plus polarity magnet ¨
  rpm = float(60 * 1000) / (float(millis() - t0)) * float(detection) / float(POLE_PAIR_NUM);
  detection = 0;
  rpmData.RPM = rpm;

  Serial.print("RPM: ");
  Serial.println(rpmData.RPM);

  return rpmData;
}

uint64_t HallSensor::detection = 0;