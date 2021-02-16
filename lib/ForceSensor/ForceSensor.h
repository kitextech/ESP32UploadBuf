#ifndef ForceSensor_h
#define ForceSensor_h

#include <Arduino.h>
#include <schema.pb.h>
#include <HX711.h>
#include <Timer.h>
#include <ProtobufBridge.h>


class ForceSensor
{
public:
  ForceSensor(int32_t ID, byte dout, byte pd_sck, float regC_, float offset_);
  void setup(ProtobufBridge bridge);
  Force prepareData(int64_t time);
  int t0;
  void loopWifiAndTime(int64_t time);


private:
  ProtobufBridge protobridge;
  HX711 sensor;
  int32_t id;
  byte dout;
  byte pd_sck;
  float regC;
  float offset;
  Timer doUpload{10};
};
#endif