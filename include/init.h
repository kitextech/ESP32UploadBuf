#include <Arduino.h>
#include <iostream>
#include <WiFi.h>
#include <TimeSync.h>
#include <ProtobufBridge.h>

#include <stdio.h>
#include "./pb_encode.h"
#include "./pb_decode.h"
#include "schema.pb.h"

#define LED_PIN 0

// Sensor and vesc include statements
#define IMU 0
#define WIND 0
#define POWER 0
#define RPM_HALL 0
#define TEMPERATURE 0

#define HAS_VESC 1

// WiFi
const char *ssid = "kitex"; // use kitexField
const char *password = "morepower";
// const char *addr = "192.168.8.144"; // black-pearl pi
const char *addr = "192.168.8.101"; // Office laptop (make static if not already...)
// const char *addr = "192.168.8.106"; // Andreas laptop