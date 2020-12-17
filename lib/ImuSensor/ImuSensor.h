#ifndef ImuSensor_h
#define ImuSensor_h

#include <Arduino.h>
#include <schema.pb.h>
#include <Adafruit_Sensor.h> // BNO-055
#include <Adafruit_BNO055.h> // BNO-055
#include <Timer.h>
#include <ProtobufBridge.h>

class ImuSensor
{
  public:
    ImuSensor();
    void setup(ProtobufBridge bridge);

    
    Timer uploadTimer{20};
    void loop();
    void loopWifiAndTime(int64_t time);


  private:
    ProtobufBridge protobridge;
    AccGyro prepareData(int64_t time);
    Adafruit_BNO055 bno = Adafruit_BNO055{-1, 0x28};
    void displaySensorDetails();
    void displaySensorStatus();
    void displayCalStatus();
    Timer statusTimer{1000};
    int64_t timeCount;
    float lastX;


};
#endif