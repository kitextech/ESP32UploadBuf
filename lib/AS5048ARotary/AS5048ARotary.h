#ifndef AS5048ARotary_h
#define AS5048ARotary_h

#include <Arduino.h>
#include <schema.pb.h>
#include <AS5048A.h>
#include <Timer.h>
#include <ProtobufBridge.h>


class AS5048ARotary
{
  public:
    AS5048ARotary();
    void setup(ProtobufBridge bridge);
    
    Timer uploadTimer{100};
    void loopWifiAndTime(int64_t time);

  private:
    ProtobufBridge protobridge;
    WindDirection prepareData(int64_t time);
    AS5048A angleSensor{33, false}; // 33 == SS
};
#endif