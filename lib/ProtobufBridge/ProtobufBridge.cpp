#include "ProtobufBridge.h"

uint8_t ProtobufBridge::buffer[128] = {0};
size_t ProtobufBridge::messageLength = 0;

bool ProtobufBridge::writeBuffer(pb_ostream_t *stream, const pb_field_iter_t *field, void *const *arg)
{
  if (!pb_encode_tag_for_field(stream, field))
    return false;

  return pb_encode_string(stream, (uint8_t *)&buffer, messageLength);
}

void ProtobufBridge::sendIMU(Imu imu) {
  // create stream from the buffer
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // write IMU data
  bool status = pb_encode(&stream, Imu_fields, &imu);
  // check the status
  if (!status)
  {
    Serial.println("Failed to encode imu");
    return;
  }
  // Serial.print("Message Length imu: ");
  // Serial.println(stream.bytes_written);
  messageLength = stream.bytes_written;

  stream = pb_ostream_from_buffer(bufferWrapper, sizeof(bufferWrapper));

  // wrapper
  Wrapper wrap = Wrapper_init_zero;
  wrap.type = Wrapper_DataType_IMU;
  wrap.data.funcs.encode = &ProtobufBridge::writeBuffer;

  status = pb_encode(&stream, Wrapper_fields, &wrap);

  if (!status)
  {
    Serial.println("Failed to encode wrapper");
    return;
  }

  wrapMessageLength = stream.bytes_written;

  // Serial.print("Message Length wrapper: ");
  Serial.println(stream.bytes_written);
}

void ProtobufBridge::sendWind(Wind wind) {
  // create stream from the buffer
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // write IMU data
  bool status = pb_encode(&stream, Wind_fields, &wind);
  // check the status
  if (!status)
  {
    Serial.println("Failed to encode wind");
    return;
  }
  // Serial.print("Message Length wind: ");
  // Serial.println(stream.bytes_written);
  messageLength = stream.bytes_written;

  stream = pb_ostream_from_buffer(bufferWrapper, sizeof(bufferWrapper));

  // wrapper
  Wrapper wrap = Wrapper_init_zero;
  wrap.type = Wrapper_DataType_WIND;
  wrap.data.funcs.encode = &ProtobufBridge::writeBuffer;

  status = pb_encode(&stream, Wrapper_fields, &wrap);

  if (!status)
  {
    Serial.println("Failed to encode wrapper");
    return;
  }

  wrapMessageLength = stream.bytes_written;

  // Serial.print("Message Length wrapper: ");
  Serial.println(stream.bytes_written);
}