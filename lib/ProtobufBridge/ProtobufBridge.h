
#ifndef ProtobufBridge_h

#define ProtobufBridge_h

#include "Arduino.h"
#include "schema.pb.h"
#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"

class ProtobufBridge
{
public:
  //variables
  static uint8_t buffer[128];
  static size_t messageLength;
  size_t wrapMessageLength;
  uint8_t bufferWrapper[512];

  // functions
  void sendIMU(Imu imu);
  void sendWind(Wind wind);
  static bool writeBuffer(pb_ostream_t *stream, const pb_field_iter_t *field, void *const *arg);

private:
};

#endif
