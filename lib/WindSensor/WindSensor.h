#ifndef WindSensor_h
#define WindSensor_h

#include <Arduino.h>
#include <schema.pb.h>
#include <MedianFilter.h>
#include <AS5040.h>

// #define PWM_MODE 1

class WindSensor
{
public:
  WindSensor(uint8_t ADC_pin, int voltDivRatio, float vMin, float vMax, float SpeedMin, float SpeedMax, bool pwmMode, int Freq);
  void setupWindDirEncoder();
  void windDirectionPWM_setup(uint8_t ADC_pin);
  Wind prepareData(int64_t time);

  int t0;
  uint16_t uploadFrequency;

private:
  MedianFilter medFilter{1, 0};
  AS5040 windDirEncoder{14, 15, 12, 13};

  float mapFloat(float value, float in_min, float in_max, float out_min, float out_max);
  float windDirection();
  float windDirectionPWM();

  int analogPin;
  int analogPWMPin;
  float voltMin;
  float voltMax;
  float speedMin;
  float speedMax;
  bool PWM_MODE;
};
#endif