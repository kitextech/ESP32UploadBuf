#ifndef ProtobufBridge_h
#define ProtobufBridge_h

#include "Arduino.h"
#include "schema.pb.h"
#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "WiFiUdp.h"

class ProtobufBridge
{
public:
  //variables and objects
  WiFiUDP *udp; // needs to be set before use
  IPAddress serverip; // needs to be set before use
  unsigned int serverport; // needs to be set before use
  unsigned int serverportBundle = 10104; // UDP bundle messages up to 10x 
  static uint8_t buffer[128];
  static size_t messageLength;
  size_t wrapMessageLength;
  uint8_t bufferWrapper[128];
  uint8_t udpBundle[1024];
  int bundleIndex;

  // functions
  void sendIMU(Imu imu);
  void sendAccGyro(AccGyro accGyro);
  void sendWind(Wind wind);
  void sendSpeed(Speed speed);
  void sendTemperature(Temperature temperature);
  void sendPower(Power power);
  void sendForce(Force force);
  void sendBladeControl(BladeControl bladeControl);
  void sendHumidityTemperature(HumidityTemperature humtemp);
  void sendWindDirection(WindDirection windDir);

  void sendVesc(Vesc vesc);
  void sendSetpoint(Setpoint setpoint);

  static bool writeBuffer(pb_ostream_t *stream, const pb_field_iter_t *field, void *const *arg);

private:
  void sendPacket();
  void sendPacketBundle();
};

#endif