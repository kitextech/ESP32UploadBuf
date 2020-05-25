#ifndef WindSensor_h
#define WindSensor_h

#include <Arduino.h>
#include <schema.pb.h>
#include <MedianFilter.h>
#include <AS5040.h>

class WindSensor
{
public:
  WindSensor(int ADC_pin, int voltDivRatio, float vMin, float vMax, float SpeedMin, float SpeedMax, int Freq);
  void setupWindDirEncoder();
  Wind prepareData(int64_t time);

  int t0;
  uint16_t uploadFrequency;

private:
  MedianFilter medFilter{1, 0};
  AS5040 windDirEncoder{14, 15, 12, 13};

  float mapFloat(float value, float in_min, float in_max, float out_min, float out_max);
  float windDirection();

  int analogPin;
  float voltMin;
  float voltMax;
  float speedMin;
  float speedMax;
};
#endif