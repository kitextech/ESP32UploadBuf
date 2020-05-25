#ifndef ImuSensor_h
#define ImuSensor_h

#include <Arduino.h>
#include <schema.pb.h>
#include <Adafruit_Sensor.h> // BNO-055
#include <Adafruit_BNO055.h> // BNO-055

#define BNO055_SAMPLERATE_DELAY_MS (10)

class ImuSensor
{
  public:
    Imu prepareData(int64_t time);
    void setup();
    
    int t0;
    uint16_t uploadFrequency;

  private:
    Adafruit_BNO055 bno = Adafruit_BNO055{-1, 0x28};
};
#endif