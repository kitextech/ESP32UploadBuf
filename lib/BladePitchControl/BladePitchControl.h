#ifndef BladePitchControl_h
#define BladePitchControl_h

// #include <Arduino.h>
#include <schema.pb.h>
#include <ESP32Servo.h>

#include "./pb_encode.h"
#include "./pb_decode.h"
// create four servo objects 
#include <WiFi.h>


class BladePitchControl
{
public:
  BladePitchControl(int pin1, int pin2, int pin3, uint16_t uploadFrequency);

  BladeControl prepareData(int64_t time);

  void setup();
  void loop(uint8_t UDPInBuffer[], int n);


  int t0;
  uint16_t uploadFrequency;

private:
  float limit(float val, float min, float max);

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