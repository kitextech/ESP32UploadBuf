#ifndef ForceSensor_h
#define ForceSensor_h

#include <Arduino.h>
#include <schema.pb.h>
#include <HX711.h>

class ForceSensor
{
public:
  ForceSensor(int32_t ID, byte dout, byte pd_sck, float regC_, float offset_, uint16_t uploadFreq_);
  void setup();
  Force prepareData(int64_t time);

  uint16_t uploadFrequency;
  int t0;

private:
  HX711 sensor;
  int32_t id;
  byte dout;
  byte pd_sck;
  float regC;
  float offset;
};
#endif