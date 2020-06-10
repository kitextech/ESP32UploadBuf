#include "ProtobufBridge.h"

uint8_t ProtobufBridge::buffer[128] = {0};
size_t ProtobufBridge::messageLength = 0;

bool ProtobufBridge::writeBuffer(pb_ostream_t *stream, const pb_field_iter_t *field, void *const *arg)
{
  if (!pb_encode_tag_for_field(stream, field))
    return false;

  return pb_encode_string(stream, (uint8_t *)&buffer, messageLength);
}


void ProtobufBridge::sendIMU(Imu imu)
{
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
  // Serial.println(stream.bytes_written);
}


void ProtobufBridge::sendWind(Wind wind)
{
  if (wind.direction != 2000.0F)
  {
    // create stream from the buffer
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    // write wind data
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
    // Serial.println(stream.bytes_written);
    // Serial.println(wind.speed);
    // Serial.println(wind.direction);
  }
}


void ProtobufBridge::sendSpeed(Speed speed)
{
  // create stream from the buffer
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // write IMU data
  bool status = pb_encode(&stream, Speed_fields, &speed);
  // check the status
  if (!status)
  {
    Serial.println("Failed to encode speed");
    return;
  }
  // Serial.print("Message Length speed: ");
  // Serial.println(stream.bytes_written);
  messageLength = stream.bytes_written;

  stream = pb_ostream_from_buffer(bufferWrapper, sizeof(bufferWrapper));

  // wrapper
  Wrapper wrap = Wrapper_init_zero;
  wrap.type = Wrapper_DataType_SPEED;
  wrap.data.funcs.encode = &ProtobufBridge::writeBuffer;

  status = pb_encode(&stream, Wrapper_fields, &wrap);

  if (!status)
  {
    Serial.println("Failed to encode wrapper");
    return;
  }

  wrapMessageLength = stream.bytes_written;

  // Serial.print("Message Length wrapper: ");
  // Serial.println(stream.bytes_written);
}


void ProtobufBridge::sendTemperature(Temperature temperature)
{
  // create stream from the buffer
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // write IMU data
  bool status = pb_encode(&stream, Temperature_fields, &temperature);
  // check the status
  if (!status)
  {
    Serial.println("Failed to encode temperature");
    return;
  }
  // Serial.print("Message Length temperature: ");
  // Serial.println(stream.bytes_written);
  messageLength = stream.bytes_written;

  stream = pb_ostream_from_buffer(bufferWrapper, sizeof(bufferWrapper));

  // wrapper
  Wrapper wrap = Wrapper_init_zero;
  wrap.type = Wrapper_DataType_TEMPERATURE;
  wrap.data.funcs.encode = &ProtobufBridge::writeBuffer;

  status = pb_encode(&stream, Wrapper_fields, &wrap);

  if (!status)
  {
    Serial.println("Failed to encode wrapper");
    return;
  }

  wrapMessageLength = stream.bytes_written;

  // Serial.print("Message Length wrapper: ");
  // Serial.println(stream.bytes_written);
}


void ProtobufBridge::sendPower(Power power)
{
  // create stream from the buffer
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // write power data
  bool status = pb_encode(&stream, Power_fields, &power);
  // check the status
  if (!status)
  {
    Serial.println("Failed to encode power");
    return;
  }
  // Serial.print("Message Length power: ");
  // Serial.println(stream.bytes_written);
  messageLength = stream.bytes_written;

  stream = pb_ostream_from_buffer(bufferWrapper, sizeof(bufferWrapper));

  // wrapper
  Wrapper wrap = Wrapper_init_zero;
  wrap.type = Wrapper_DataType_POWER;
  wrap.data.funcs.encode = &ProtobufBridge::writeBuffer;

  status = pb_encode(&stream, Wrapper_fields, &wrap);

  if (!status)
  {
    Serial.println("Failed to encode wrapper");
    return;
  }

  wrapMessageLength = stream.bytes_written;

  // Serial.print("Message Length wrapper: ");
  // Serial.println(stream.bytes_written);
}


void ProtobufBridge::sendForce(Force force)
{
  // create stream from the buffer
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // write force data
  bool status = pb_encode(&stream, Force_fields, &force);
  // check the status
  if (!status)
  {
    Serial.println("Failed to encode force");
    return;
  }
  // Serial.print("Message Length force: ");
  // Serial.println(stream.bytes_written);
  messageLength = stream.bytes_written;

  stream = pb_ostream_from_buffer(bufferWrapper, sizeof(bufferWrapper));

  // wrapper
  Wrapper wrap = Wrapper_init_zero;
  wrap.type = Wrapper_DataType_FORCE;
  wrap.data.funcs.encode = &ProtobufBridge::writeBuffer;

  status = pb_encode(&stream, Wrapper_fields, &wrap);

  if (!status)
  {
    Serial.println("Failed to encode wrapper");
    return;
  }

  wrapMessageLength = stream.bytes_written;

  // Serial.print("Message Length wrapper: ");
  // Serial.println(stream.bytes_written);
}


void ProtobufBridge::sendVesc(Vesc vesc)
{
  // create stream from the buffer
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // write vesc data
  bool status = pb_encode(&stream, Vesc_fields, &vesc);
  // check the status
  if (!status)
  {
    Serial.println("Failed to encode vesc");
    return;
  }
  // Serial.print("Message Length vesc: ");
  // Serial.println(stream.bytes_written);
  messageLength = stream.bytes_written;

  stream = pb_ostream_from_buffer(bufferWrapper, sizeof(bufferWrapper));

  // wrapper
  Wrapper wrap = Wrapper_init_zero;
  wrap.type = Wrapper_DataType_VESC;
  wrap.data.funcs.encode = &ProtobufBridge::writeBuffer;

  status = pb_encode(&stream, Wrapper_fields, &wrap);

  if (!status)
  {
    Serial.println("Failed to encode wrapper");
    return;
  }

  wrapMessageLength = stream.bytes_written;

  // Serial.print("Message Length wrapper: ");
  // Serial.println(stream.bytes_written);
}


void ProtobufBridge::sendSetpoint(Setpoint setpoint)
{
  // create stream from the buffer
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // write setpoint data
  bool status = pb_encode(&stream, Setpoint_fields, &setpoint);
  // check the status
  if (!status)
  {
    Serial.println("Failed to encode setpoint");
    return;
  }
  // Serial.print("Message Length setpoint: ");
  // Serial.println(stream.bytes_written);
  messageLength = stream.bytes_written;

  stream = pb_ostream_from_buffer(bufferWrapper, sizeof(bufferWrapper));

  // wrapper
  Wrapper wrap = Wrapper_init_zero;
  wrap.type = Wrapper_DataType_SETPOINT;
  wrap.data.funcs.encode = &ProtobufBridge::writeBuffer;

  status = pb_encode(&stream, Wrapper_fields, &wrap);

  if (!status)
  {
    Serial.println("Failed to encode wrapper");
    return;
  }

  wrapMessageLength = stream.bytes_written;

  // Serial.print("Message Length wrapper: ");
  // Serial.println(stream.bytes_written);
}