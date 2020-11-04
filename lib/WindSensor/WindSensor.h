#ifndef WindSensor_h
#define WindSensor_h

#include <Arduino.h>
#include <schema.pb.h>
#include <AS5040.h>
#include <Timer.h>
#include <ProtobufBridge.h>


// #define PWM_MODE 1

class WindSensor
{
public:
  WindSensor(uint8_t ADC_pin);
  
  void setup(ProtobufBridge bridge);

  // void setupWindDirEncoder();
  // void windDirectionPWM_setup(uint8_t ADC_pin);
  Wind prepareData(int64_t time);

  Timer uploadTimer{300};
  void loop();
  void loopWifiAndTime(int64_t time);

private:
  // AS5040 windDirEncoder{14, 15, 12, 13};


  float mapFloat(float value, float in_min, float in_max, float out_min, float out_max);
  float windDirection();
  float windDirectionPWM();

  ProtobufBridge protobridge;

  int analogPin;
  int analogPWMPin;
  float voltMin;
  float voltMax;
  float speedMin;
  float speedMax;
  bool PWM_MODE;
};
#endif