#ifndef HallSensor_h
#define HallSensor_h

#include <Arduino.h>
#include <schema.pb.h>

// #define PIN_HALL 2          // D4 pin on ESP8266 NodeMCU for one hall sensor connection
// #define POLE_PAIR_NUM 7     // 14 poles -> 7 pole pairs, counted manually

class HallSensor
{
  public:
    HallSensor(uint8_t pinHall, uint8_t polePairNum, uint16_t uploadFreq);
    void setup();
    Speed prepareData(int64_t time);
    
    int t0;
    uint16_t uploadFrequency;

  private:
    static ICACHE_RAM_ATTR void magnet_detect();
    static uint64_t detection;

    float rpm = 0;
    uint8_t PIN_HALL;
    uint8_t POLE_PAIR_NUM;
};
#endif