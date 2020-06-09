#include <Arduino.h>
#include <iostream>
#include <WiFi.h>
#include <TimeSync.h>
#include <ProtobufBridge.h>

#include <stdio.h>
#include "./pb_encode.h"
#include "./pb_decode.h"
#include "schema.pb.h"

#include <PID_v1.h>

#include <queue>
#include <list>

using namespace std; 

#define LED_PIN 0

// Sensor and vesc include statements
#define IMU 0
#define WIND 0
#define POWER 0
#define POWER_DUMP 0
#define RPM_HALL 0
#define TEMPERATURE 0
#define FORCE 1

#define HAS_VESC 0

#if IMU
#include <ImuSensor.h>
ImuSensor imuSensor(5);
#endif
#if WIND
#include <WindSensor.h>
WindSensor windSensor(A2, 2, 0.4, 2, 0.2, 32.4, true, 3);
#endif

#if (POWER && POWER_DUMP)
#include <PowerSensor.h>
PowerSensor powerSensor(50, A2, A3, 0.12, 1, 0.8305, 0.7123, 0, 18.01, -1.866, 28.6856, 1, 3.2, 4.0, A9, A10, A11, A12);
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
    ForceSensor(1, A1, A0, -9.479e-005, -0.3162, 10),
    ForceSensor(2, A11, A12, -9.462e-005, 1.444, 10),
    ForceSensor(3, A9, A10, -9.332e-005, -0.7352, 10)
};
#endif

#if HAS_VESC
#include <HardwareSerial.h>
#include <VescUart.h>

#define MODE_ARRAY_LENGTH 5

HardwareSerial SerialVesc(2);
VescUart vesc;
int t0_Vesc = millis();
int uploadFreqVesc = 30;
int t0_ramp;
float rpmDiff;
int rampingTime = 3000;
float rpm_sp = 0.0;

float maxCurrent = 5;
float minCurrent = -45;
float rpmSetpoint = 0.0;

int rpmSetpointArray[MODE_ARRAY_LENGTH] = {0}; 

float pidSUM = 0;

int tcpPort = 10101;
WiFiServer server(tcpPort);
WiFiClient client = server.available();
uint8_t bufferTCP[128] = {0};
#endif

// WiFi
const char *ssid = "kitexField"; // use kitexField
const char *password = "morepower";
// const char *addr = "192.168.8.144"; // black-pearl pi
const char *addr = "192.168.8.107"; // Office laptop (make static if not already...)
// const char *addr = "192.168.8.106"; // Andreas laptop

// Time and udp setup
IPAddress timeServerIP;
unsigned int udpPortLocal = 2390;
TimeSync timeSync;
int64_t baseTime;
int64_t sysTimeAtBaseTime;
const uint32_t secondsUntilNewTime = 300;

IPAddress insertServerIP;
unsigned int udpPortRemoteInsert = 10102;
WiFiUDP udp;
ProtobufBridge protobufBridge;