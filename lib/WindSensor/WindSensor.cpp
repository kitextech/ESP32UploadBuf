#include "WindSensor.h"

WindSensor::WindSensor(uint8_t ADC_pin, int voltDivRatio, float vMin, float vMax, float SpeedMin, float SpeedMax, int Freq)
{
  t0 = millis();
  voltMin = vMin / voltDivRatio;
  voltMax = vMax / voltDivRatio;
  analogPin = ADC_pin;
  speedMin = SpeedMin;
  speedMax = SpeedMax;
  uploadFrequency = Freq;
  // analogPin = analogPin;
  Serial.println("Created a Wind sensor");
}

void WindSensor::setupWindDirEncoder()
{
  // connect to the AS5140 sensor
  if (!windDirEncoder.begin())
  {
    Serial.println("Error setting up wind direction encoder");
  }
  else
  {
    Serial.println("Successfully set up wind direction encoder");
  }
}

float WindSensor::mapFloat(float value, float in_min, float in_max, float out_min, float out_max)
{
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float WindSensor::windDirection()
{
  int rawData = windDirEncoder.read();
  bool Status = windDirEncoder.valid();
  float direction = 0.0F;

  if (Status == true)
  {
    direction = mapFloat(rawData, 0, 1024, 0, 360);
  }
  else
  {
    direction = (float)2000;
  }

  return direction;
}

Wind WindSensor::prepareData(int64_t time)
{
  Wind windData = Wind_init_zero;
  windData.time = time;

  medFilter.in(analogRead(analogPin));
  float rawSensorData = (float)medFilter.out();
  // map the raw sensor data to the voltage between 0.0 to 3.0
  float sensorValue = mapFloat(analogRead(analogPin), 95.0F, 1024.0F, 0.195F, 1.0F);
  /* 
     * convert the voltage to wind speed 
     * calibration refrence 
     * https://thepihut.com/products/adafruit-anemometer-wind-speed-sensor-w-analog-voltage-output
  */
  // float measuredSpeed = mapFloat(sensorValue, voltMin, voltMax, speedMin, speedMax);
  // Serial.print("wind speed : \t");
  // Serial.println(measuredSpeed);

  windData.speed = mapFloat(sensorValue, voltMin, voltMax, speedMin, speedMax);
  windData.direction = windDirection();
  /*
  Serial.print("Wind : ");
  Serial.println(windData.speed);
  Serial.print("Analog read : ");
  Serial.println(analogRead(analogPin));
  Serial.print("Raw Volt : ");
  Serial.println(sensorValue);
  */
  return windData;
}