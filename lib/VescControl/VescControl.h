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
#include "SpeedController.h"

#define RPM_TO_ERPM 163.8f


class VescControl
{
public:
  VescControl(boolean doDump);
  void setup(ProtobufBridge proto);
  void updateTurbineControl(uint8_t UDPInBuffer[], int n);
  void loopWifiAndTime(int64_t time);
  void loop();

  Timer readVesc{100};
  Timer uploadData{100};
  Timer controlDump{1000};

private:
  HardwareSerial SerialVesc{2};
  VescUart vesc;
  Vesc prepareVescData(int64_t time);
  Setpoint prepareSetpointData(int64_t time);
  void controlVESC();
  void controlDumping(float voltage);
  TurbineControl control;
  ProtobufBridge protobridge;
  float rpmsetpoint;
  float currentSetpoint;
  boolean doDump;
  SpeedController speedController;

};
#endif