#ifndef TimeSync_h
    #define TimeSync_h
    #include <ESP8266WiFi.h>
    #include "Arduino.h"

    #include <WiFiUdp.h> // NTC

    
    const int NTP_PACKET_SIZE = 48;
    // const char* ntpServerName = "dk.pool.ntp.org"; //"time.nist.gov";

    class TimeSync
    {
        public:
            // Time();
            // WiFiUDP udp;
            
            // const int NTP_PACKET_SIZE = 48;
            const char* ntpServerName = "dk.pool.ntp.org"; //"time.nist.gov";
            byte packetBuffer[NTP_PACKET_SIZE];

            int64_t getTime(IPAddress timeServerIP, WiFiUDP udp);
            unsigned long sendNTPpacket(IPAddress& address, WiFiUDP udp);
            int64_t Parse(struct parsedTime *timeStruct, byte* packet);

    };

#endif