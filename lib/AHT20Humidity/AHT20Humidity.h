#ifndef AHT20Humidity_h
#define AHT20Humidity_h

#include <Arduino.h>
#include <schema.pb.h>
#include <Adafruit_AHTX0.h>
#include <Timer.h>
#include <ProtobufBridge.h>


class AHT20Humidity
{
  public:
    AHT20Humidity();
    void setup(ProtobufBridge bridge);
    
    Timer uploadTimer{2000};
    void loopWifiAndTime(int64_t time);

  private:
    ProtobufBridge protobridge;
    HumidityTemperature prepareData(int64_t time);
    Adafruit_AHTX0 aht;
};
#endif