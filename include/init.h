#include <Arduino.h>
#include <iostream>
#include <WiFi.h>
#include <time.h>
#include <ProtobufBridge.h>

#include <stdio.h>
#include "./pb_encode.h"
#include "./pb_decode.h"
#include "schema.pb.h"

using namespace std;

#define LED_PIN 0

// Sensor and vesc include statements
#define IMU 0
#define WIND 0
#define POWER 0
#define POWER_DUMP 0
#define RPM_HALL 0
#define TEMPERATURE 0
#define FORCE 0
#define OLED 0
#define VESC 0
#define BLADE 1

#if IMU
#include <ImuSensor.h>
ImuSensor imuSensor(40);
#endif
#if WIND
#include <WindSensor.h>
WindSensor windSensor(A2, 2, 0.4, 2, 0.2, 32.4, true, 3);
#endif
#if (POWER && POWER_DUMP)
#include <PowerSensor.h>
//PowerSensor powerSensor(50, A2, A3, 0.12, 1, 0.8305, 0.7123, 0, 18.01, -1.866, 28.6856, 1, 28.8, 36.0, A9, A10, A7, A12);
// we need to remove three of the gates
PowerSensor powerSensor(50, A3, A2, 0.12, 1, 0.8305, 0.7123, 0, 18.01, -1.866, 28.6856, 1, 12.0, 14.0, A9, A10, A7, A12);
#endif
#if (POWER && !POWER_DUMP)
#include <PowerSensor.h>
PowerSensor powerSensor(50, A2, A3, 0.12, 1, 0.8305, 0.7123, 0, 18.01, -1.866, 28.6856, 1);
#endif
#if RPM_HALL
#include <HallSensor.h>
HallSensor hallSensor(2, 7, 2);
#endif
#if TEMPERATURE
#include <TemperatureSensor.h>
TemperatureSensor temperatureSensor(1, A0, 10000, 25, 3950, 10000);
#endif
#if FORCE
#include <ForceSensor.h>
ForceSensor forceSensors[] = {
    ForceSensor(1, A1, A0, -9.479e-005, -0.3162, 40),
    ForceSensor(2, A11, A12, -9.462e-005, 1.444, 40),
    ForceSensor(3, A9, A10, -9.332e-005, -0.7352, 40)};
#endif
#if OLED
#include <Oled.h>
Oled oled(5);
#endif
#if VESC
#include <VescControl.h>
VescControl vescControl(30);

int tcpPort = 10101;
WiFiServer server(tcpPort);
WiFiClient client = server.available();
// uint8_t bufferTCP[128] = {0};
#endif

#if BLADE

#include <BladePitchControl.h>
// These are all GPIO pins on the ESP32
// Recommended pins include 2,4,12-19,21-23,25-27,32-33 
int servo1Pin = 33;
int servo2Pin = 15;
int servo3Pin = 32;
BladePitchControl bladePitchControl(servo1Pin, servo2Pin, servo3Pin, 1);
#endif

// WiFi
const char *ssid = "kitex"; // use kitexField
const char *password = "morepower";
const char *addr = "192.168.8.152"; // Andreas' laptop on kitex
// const char *addr = "192.168.8.126"; // Andreas' laptop on kitexField

// send upd data
IPAddress insertServerIP;
unsigned int udpPortRemoteInsert = 10102;
WiFiUDP udp;
ProtobufBridge protobufBridge;

// recieve upd data
unsigned int udpPortLocalRecieve = 10102;
uint8_t UDPInBuffer[128]; 
