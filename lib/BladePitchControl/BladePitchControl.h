#ifndef BladePitchControl_h
#define BladePitchControl_h

// #include <Arduino.h>
#include <schema.pb.h>
#include <ESP32Servo.h>

#include "./pb_encode.h"
#include "./pb_decode.h"
// create four servo objects 
#include <WiFi.h>
#include <Timer.h>
#include <ProtobufBridge.h>


class BladePitchControl
{
public:
  BladePitchControl(int pin1, int pin2, int pin3);

  Timer uploadTimer{100};
  void setup(ProtobufBridge bridge);
  void loop(uint8_t UDPInBuffer[], int n);
  void loopWifiAndTime(int64_t time);

private:
  float limit(float val, float min, float max);
  ProtobufBridge protobridge;
  BladeControl prepareData(int64_t time);
  int pin1;
  int pin2;
  int pin3;

  float pitch1;
  float pitch2;
  float pitch3;
  float collectivePitch;

  Servo servo1;
  Servo servo2;
  Servo servo3;
};
#endif