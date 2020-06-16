#include "Oled.h"

Oled::Oled(int uploadFreq)
{
  t0 = millis();
  updateFrequency = uploadFreq;
  Serial.println("Created an OLED handling object");
}

void Oled::setup()
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);  
}

void Oled::displayTime(int64_t time)
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }

  display.clearDisplay();
  display.setCursor(0,10);
  display.print(&timeinfo, "%H");
  display.print(&timeinfo, "%M");
  display.print(":");
  display.println(&timeinfo, "%S");
  display.display();
}