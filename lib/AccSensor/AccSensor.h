#ifndef AccSensor_h
#define AccSensor_h

#include <Arduino.h>
#include <schema.pb.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Timer.h>


class AccSensor
{
  public:
    AccSensor(uint16_t updatePeriod);
    Imu prepareData(int64_t time);
    void setup();
    
    Timer doUpload;

  private:
  Adafruit_MMA8451 mma = Adafruit_MMA8451();
};
#endif