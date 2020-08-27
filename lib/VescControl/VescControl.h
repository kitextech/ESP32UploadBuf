#ifndef VescControl_h
#define VescControl_h

#include <Arduino.h>
#include <schema.pb.h>
#include <HardwareSerial.h>
#include <VescUart.h>
#include <WiFi.h>
#include <ProtobufBridge.h>
#include <Timer.h>

#include "./pb_encode.h"
#include "./pb_decode.h"
#include <ArduinoJson.h>

#include "esp_http_client.h" // not in use, but might be relevant if we want ASYNC HTTP requests
#include <HTTPClient.h>
#include <AsyncTCP.h>

class VescControl
{
public:
  VescControl(uint16_t uploadFreq);
  void setup();
  Vesc prepareVescData(int64_t time);
  Setpoint prepareSetpointData(int64_t time);
  void updateRpmSetpoint(uint8_t UDPInBuffer[], int n);
  void updateRpmSetpoint(float RPM);
  void setRpm();
  void runFirebaseCheck();
  void runFirebaseCheckAsync();
  void runAsyncClient();
  String httpGETRequest();

  int t0;
  int t0_ramp;
  float rpmDiff;
  int rampingTime = 3000;
  float rpm_sp = 0;
  float maxCurrent = 5;
  float minCurrent = -45;
  float rpmSetpoint = 0.0;
  float rpmRampStart = 0;
  float rampAcc = .7; // RPM/ms^2
  Timer checkFirebase;
  float rpm_sp_udp;
  uint16_t uploadFrequency;
  float turbineGearRatio = 163.8; // 23.4*7


private:
  uint8_t modeArrayLength;
  HardwareSerial SerialVesc{2};
  VescUart vesc;
  StaticJsonDocument<64> firebaseControlDoc;
  HTTPClient http; 

};
#endif