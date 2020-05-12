#include "TimeSync.h"

struct parsedTime {
  int hour;
  int minute;
  int second;
  uint16_t milisecond;
};

struct parsedTime Ptime;

// send an NTP request to the time server at the given address
void TimeSync::sendNTPpacket(IPAddress& address, WiFiUDP udp)
{
  // Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

int64_t TimeSync::Parse(struct parsedTime *timeStruct, byte* packet){
   // make the POSIX time
   unsigned long highWord = word(packet[40], packet[41]);
   unsigned long lowWord  = word(packet[42], packet[43]);
   unsigned long secsSince1900 = highWord << 16 | lowWord;
   // convert to NTP time
   const unsigned long seventyYears = 2208988800UL; 
   unsigned long epoch = secsSince1900 - seventyYears;
   // get the hour,minute,seconds
    timeStruct->hour   = (epoch % 86400L) / 3600;         
    timeStruct->minute = (epoch % 3600) / 60;
    timeStruct->second = (epoch % 60);
    // get the fraction of seconds 
    // https://arduino.stackexchange.com/questions/49567/synching-local-clock-usign-ntp-to-milliseconds
    uint32_t frac  = (uint32_t) packet[44] << 24
                   | (uint32_t) packet[45] << 16
                   | (uint32_t) packet[46] <<  8
                   | (uint32_t) packet[47] <<  0;
    timeStruct->milisecond = ((uint64_t) frac * 1000 ) >> 32;


    return int64_t( int64_t(epoch) * int64_t(1000) + int64_t( timeStruct->milisecond ) );
}


int64_t TimeSync::getTime(IPAddress timeServerIP, WiFiUDP udp) {
  // bool gotTheTime = false;

  while (true) {
    WiFi.hostByName(timeServerName, timeServerIP); // timeServerName
    sendNTPpacket(timeServerIP, udp); // send an NTP packet to a time server
    
    delay(500);
    int cb = udp.parsePacket();
    if (!cb) {
      int t0 = millis();
      int t1 = millis();
      int dt = 0;
      while (millis()-t0 < 500) {
        if (dt / 100) {
          digitalWrite(0, HIGH);
          t1 = millis();
        } else if (dt / 75) {
          digitalWrite(0, LOW);
        } else {
          digitalWrite(0, HIGH);
        }
        dt = millis()-t1;
      }
      digitalWrite(0, LOW);
      Serial.println("no packet yet");
    }
    else {
      udp.read(packetBuffer, NTP_PACKET_SIZE);
      int64_t time = Parse(&Ptime, packetBuffer);
      // print in serial port 
      Serial.println("Got the time: ");
      Serial.print (Ptime.hour);
      Serial.print (":");
      Serial.print (Ptime.minute);
      Serial.print (":");
      Serial.print (Ptime.second);
      Serial.print (":");
      Serial.println (Ptime.milisecond);
      return time;
    }
  }
  // Serial.println ( "PROBLEM!");   
  // return 0;
}
