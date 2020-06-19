#ifndef PowerSensor_h
#define PowerSensor_h

#include <Arduino.h>
#include <schema.pb.h>

class PowerSensor
{
public:
  PowerSensor(uint8_t numSamples_, uint8_t voltagePin_, uint8_t currentPin_,
              float b1_, float m1_, float b2_, float m2_, float bV_, float mV_, float bC_, float mC_,
              uint16_t uploadFrequency_);

  PowerSensor(uint8_t nSamples, uint8_t vPin, uint8_t cPin,
              float b1_, float m1_, float b2_, float m2_, float bV_, float mV_, float bC_, float mC_,
              uint16_t uploadFrequency_, float minVolt, float maxVolt, uint8_t dumpPin1, uint8_t dumpPin2, uint8_t dumpPin3, uint8_t dumpPin4);

  Power prepareData(int64_t time);

  void PowerDumpSetup();
  void PowerControl();

  uint16_t uploadFrequency;
  int t0;
  float voltage = 0;

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

  int sumC = 0;
  int sumV = 0;

  float BatMinVolt = 0;
  float BatMaxVolt = 0;
  int DigitalPin1;
  int DigitalPin2;
  int DigitalPin3;
  int DigitalPin4;
};
#endif