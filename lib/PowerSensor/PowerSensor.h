#ifndef PowerSensor_h
#define PowerSensor_h

#include <Arduino.h>
#include <schema.pb.h>

class PowerSensor
{
  public:
    PowerSensor(uint8_t numSamples, uint8_t voltagePin, uint8_t currentPin, \
      float b1, float m1, float b2, float m2, float bV, float mV, float bC, float mC, \
      uint16_t uploadFrequency);

    Power prepareData(int64_t time);
    
    uint16_t uploadFrequency;
    int t0;

  private:
    uint8_t numSamples;
    uint8_t voltagePin;
    uint8_t currentPin;
    float b1;
    float m1;
    float b2;
    float m2;
    float bV;
    float mV;
    float bC;
    float mC;

    float adcCurrent = 0;
    float adcVoltage = 0;
    float current = 0;
    float voltage = 0;
    
    int sumC = 0;
    int sumV = 0;
};
#endif