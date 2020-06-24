#ifndef VescControl_h
#define VescControl_h

#include <Arduino.h>
#include <schema.pb.h>
#include <HardwareSerial.h>
#include <VescUart.h>
#include <WiFi.h>
#include <ProtobufBridge.h>

#include "./pb_encode.h"
#include "./pb_decode.h"

#define MODE_ARRAY_LENGTH 5

class VescControl
{
public:
  VescControl(uint16_t uploadFreq);
  void setup();
  Vesc prepareVescData(int64_t time);
  Setpoint prepareSetpointData(int64_t time);
  int mode(int a[], int n);
  void updateArray(int newElement, int n);
  void updateRpmSetpoint(WiFiClient client);
  void setRpm();

  int t0;
  uint16_t uploadFrequency;
  int t0_ramp;
  float rpmDiff;
  int rampingTime = 3000;
  float rpm_sp = 0;
  float maxCurrent = 5;
  float minCurrent = -45;
  float rpmSetpoint = 0.0;
  float rampAcc = .7; // RPM/ms^2
  int rpmSetpointArray[MODE_ARRAY_LENGTH] = {0};
  float pidSUM = 0;

private:
  uint8_t modeArrayLength;
  HardwareSerial SerialVesc{2};
  VescUart vesc;
  uint8_t bufferTCP[128] = {0};
};
#endif