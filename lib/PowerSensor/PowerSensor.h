#ifndef PowerSensor_h
#define PowerSensor_h

#include <Arduino.h>
#include <schema.pb.h>
#include <Timer.h>

class PowerSensor
{
public:
  enum Indicator
  {
    AUTOMATIC_DUMP_ON,
    AUTOMATIC_DUMP_OFF,
    MANUAL_DUMP_OFF,
    WARNING
  } status_indicator;

  PowerSensor(uint8_t numSamples_, uint8_t voltagePin_, uint8_t currentPin_,
              float b1_, float m1_, float b2_, float m2_, float bV_, float mV_, float bC_, float mC_,
              uint16_t uploadFrequency_, uint16_t nofiLoopPeriod);

  PowerSensor(uint8_t nSamples, uint8_t vPin, uint8_t cPin,
              float b1_, float m1_, float b2_, float m2_, float bV_, float mV_, float bC_, float mC_,
              uint16_t uploadFrequency_, float minVolt, float maxVolt, uint8_t dumpPin1, uint8_t dumpPin2, uint8_t dumpPin3, uint8_t dumpPin4, uint8_t chargePin, uint16_t nofiLoopPeriod);
  
  Power prepareData(int64_t time);
  void readVoltageCurrent();
  void PowerDumpSetup();
  void PowerControl();
  void Indicator();
  void chargeOnOff();

  int Blink(int time);

  uint16_t uploadFrequency;
  int t0;
  float voltage = 0;

  Timer noWifiLoopTimer;

private:
  uint8_t numSamples;
  uint8_t voltagePin;
  uint8_t currentPin;
  uint8_t chargePin;
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
  int MANUAL_OVERRIDE_SWITCH;
  int GREEN_LED;
  int RED_LED;
  int DigitalPin4;
};
#endif