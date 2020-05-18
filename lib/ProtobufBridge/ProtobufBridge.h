
#ifndef ProtobufBridge_h

#define ProtobufBridge_h

#include "Arduino.h"
#include "schema.pb.h"
#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"
#include <WiFi.h>

class ProtobufBridge
{
public:
  //variables and objects
  static uint8_t buffer[128];
  static size_t messageLength;
  size_t wrapMessageLength;
  uint8_t bufferWrapper[512];
  WiFiUDP udp;

  // functions
  void sendIMU(Imu imu);
  void sendWind(Wind wind);
  void sendSpeed(Speed speed);
  void sendTemperature(Temperature temperature);
  static bool writeBuffer(pb_ostream_t *stream, const pb_field_iter_t *field, void *const *arg);

private:
};

#endif
