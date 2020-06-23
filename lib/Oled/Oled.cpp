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
  display.setTextColor(SSD1306_WHITE);  
  display.setTextSize(2);
  display.clearDisplay();
}

void Oled::displayTime(int64_t time)
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }

  display.clearDisplay();
  display.setCursor(4,0);
  display.print(&timeinfo, "%d");
  display.print(&timeinfo, "%B");
  display.println(&timeinfo, "%Y");
  display.setCursor(15,18);
  display.print(&timeinfo, "%H");
  display.print(":");
  display.print(&timeinfo, "%M");
  display.print(":");
  display.println(&timeinfo, "%S");
  display.display();
}

void Oled::displayIP(const char *addr)
{
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("No access:");
  display.println(addr + 3);
  display.display();
}

void Oled::displayWifi(const char *ssid)
{
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Wifi name:");
  display.println(ssid);
  display.display();
}