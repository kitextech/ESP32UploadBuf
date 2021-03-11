#ifndef Util_h
#define Util_h

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Timer.h>


class WifiReconnect
{
public:
  boolean checkWifi(const char *ssid);

private:
  // Timer for reporting wifi and time status
    Timer wifiReconnect{1000}; // 10000 ms period
    Timer wifiReport{1000}; // 10000 ms period


    int wifiReconnectCount;
    void reconnectToWifi();

};
void printLocalTime();
bool setupOTA(const char *hostname);


#endif