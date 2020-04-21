#include <Arduino.h>
// #include "msg.pb.h"
// #include "schema.pb.h"
// #include "pb_common.h"
// #include "pb.h"
// #include "pb_encode.h"

// #include <WiFi.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h> // NTC
#include <TimeSync.h>
#include <Adafruit_Sensor.h>  // BNO-055
#include <Adafruit_BNO055.h>  // BNO-055
#include <ProtobufBridge.h>


const char* ssid     = "kitex";
const char* password = "morepower";

uint8_t buffer[128];
size_t imuMessageLength;
size_t wrapMessageLength;
uint8_t bufferWrapper[512];

WiFiClient client;
ProtobufBridge protobufBridgeIMU;

const char* addr     = "192.168.8.126";
const uint16_t port  = 10101;

// NTC
IPAddress timeServerIP;
WiFiUDP udp;
unsigned int localPort = 2390;

// Time
TimeSync timeSync;
int64_t baseTime;
int64_t sysTimeAtBaseTime;

// BNO-055
#define BNO055_SAMPLERATE_DELAY_MS (10)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

void setupIMU() { // BNO-055 SETUP
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");
  bno.setExtCrystalUse(true);
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  delay(10);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // NTC
  // connect to udp 
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  // Serial.println(up);

  baseTime = timeSync.getTime(timeServerIP, udp);
  sysTimeAtBaseTime = int64_t(millis());

  setupIMU();
}

int64_t getNewTime() {
  return baseTime - sysTimeAtBaseTime + int64_t(millis());
}

Imu prepareIMUData() {
  Imu imuData = Imu_init_zero;

  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Quaternion quat = bno.getQuat();

  imuData.has_acc = true;
  imuData.acc.x = acc.x();
  imuData.acc.y = acc.y();
  imuData.acc.z = acc.z();

  imuData.has_gyro = true;
  imuData.gyro.x = gyro[0];
  imuData.gyro.y = gyro.y();
  imuData.gyro.z = gyro.z();

  imuData.has_orientation = true;
  imuData.orientation.x = quat.x();
  imuData.orientation.y = quat.y();
  imuData.orientation.z = quat.z();
  imuData.orientation.w = quat.w();

  imuData.time = getNewTime();

  return imuData;
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);
  //Serial.print("connecting to ");
  //Serial.println(addr);

  if (!client.connected()) {
    client.connect(addr, port);
    Serial.println("connection failed");
    Serial.println("wait 5 sec to reconnect...");
    delay(5000);
    return;
  }

  Imu imuData = prepareIMUData();

  protobufBridgeIMU.sendIMU(imuData);
  client.write(protobufBridgeIMU.bufferWrapper, protobufBridgeIMU.wrapMessageLength);

  digitalWrite(LED_BUILTIN, HIGH);

  delay(20);
}