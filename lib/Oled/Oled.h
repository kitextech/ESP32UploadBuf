#ifndef Oled_h
#define Oled_h

#include <Arduino.h>
#include <schema.pb.h>
#include <Adafruit_SSD1306.h>


class Oled
{
public:
  Oled(int updateFrequency);

  void setup();
  void displayTime(int64_t time);

  int t0;
  uint16_t updateFrequency;

private:
  Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
};
#endif