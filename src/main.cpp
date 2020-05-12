#include <Arduino.h>
#include <iostream>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h> // NTC
#include <TimeSync.h>
#include <Adafruit_Sensor.h> // BNO-055
#include <Adafruit_BNO055.h> // BNO-055
#include <ProtobufBridge.h>
#include <MedianFilter.h>
#include <AS5040.h> // wind vane

/*
 * AS5040.h          -> library for reading data through SSI protocol from AS5140 magnetic sensor
 * MedianFilter.h    -> library for filtering the noisy recieved analog data
 * ESP8266WiFi.h     -> library for handling modularity of wifi on ESP8266 
 * WiFiUdp.h         -> library for reading time by UDP protocol from NTP server
 */

// WiFi and server
const char *ssid = "kitexField";
const char *password = "morepower";
const char *addr = "192.168.8.144"; // Local IP of the black-pearl pi
// const char *addr = "192.168.8.104"; // Local IP of office laptop

// Time
IPAddress timeServerIP;
WiFiUDP udp_time;
unsigned int udpPortLocalTime = 2390;

TimeSync timeSync;
int64_t baseTime;
int64_t sysTimeAtBaseTime;
const int secondsUntilNewTime = 300;

// Upload
int uploadFrequency = 2; // Hz
int t0 = millis();

IPAddress insertServerIP;
WiFiUDP udp_insert;
unsigned int udpPortRemoteInsert = 10102;

ProtobufBridge protobufBridge;

// BNO-055
#define BNO055_SAMPLERATE_DELAY_MS (10)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

// Wind direction (AS5140-H)
const int CSpin = 15;
const int CLKpin = 14;
const int DOpin = 12;
const int PROGpin = 13;
AS5040 encoder(CLKpin, CSpin, DOpin, PROGpin);

// Wind speed (analog read)
const int AnalogPin = A0;
const int VoltdivRatio = 2;
const float minVoltage = 0.4 / VoltdivRatio;
const float maxVoltage = 2.0 / VoltdivRatio;
const float minSpeed = 0.2;
const float maxSpeed = 32.4;
const int updateFreq = 5;
MedianFilter MedFilter(1, 0);

// Define the LED pin - different for different ESP's
// #define LED_PIN LED_BUILTIN // For normal Arduino, possibly other ESP's
#define LED_PIN 0

float mapFloat(float value, float in_min, float in_max, float out_min, float out_max)
{
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setupIMU()
{ // BNO-055 SETUP
  Serial.println("Orientation Sensor Raw Data Test");
  Serial.println("");
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
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

void setupAS5140()
{
  // connect to the AS5140 sensor
  if (!encoder.begin())
  {
    Serial.println("Error setting up AS5140");
  }
  else
  {
    Serial.println("Successfully setting up AS5140");
  }
}
float getAS5140_data()
{
  int rawData = encoder.read();
  return mapFloat(rawData, 0, 1024, 0, 360);
}

int64_t newLocalTime()
{
  return baseTime - sysTimeAtBaseTime + int64_t(millis());
}

Imu prepareIMUData()
{
  Imu imuData = Imu_init_zero;

  imuData.time = newLocalTime();

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

  return imuData;
}

Wind prepareWindData()
{
  Wind windData = Wind_init_zero;

  windData.time = newLocalTime();

  MedFilter.in(analogRead(AnalogPin));
  int rawSensorData = MedFilter.out();
  // map the raw sensor data to the voltage between 0.0 to 3.0
  float sensorValue = mapFloat(float(rawSensorData), 6.0, 1024.0, 0.0, 1.0);
  /* 
     * convert the voltage to wind speed 
     * calibration refrence 
     * https://thepihut.com/products/adafruit-anemometer-wind-speed-sensor-w-analog-voltage-output
    */
  float measuredSpeed = mapFloat(sensorValue, minVoltage, maxVoltage, minSpeed, maxSpeed);
  // Serial.print("wind speed : \t");
  // Serial.println(measuredSpeed);

  windData.speed = measuredSpeed;
  windData.direction = getAS5140_data();
  Serial.print(windData.direction);
  return windData;
}

void getTime()
{
  Serial.println("I shall now fetch the time!");
  baseTime = timeSync.getTime(timeServerIP, udp);
  sysTimeAtBaseTime = int64_t(millis());
}

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  pinMode(LED_PIN, OUTPUT);

  delay(10);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.printf("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

  WiFi.hostByName(addr, insertServerIP); // Define IPAddress object with the ip address string

  // AS5140H
  setupAS5140();

  udp_insert.begin(udpPortRemoteInsert);

  // NTC
  // connect to udp_time
  Serial.println("Starting UDP");
  udp_time.begin(udpPortLocalTime);
  Serial.print("Local port: ");
  // Serial.println(up);

  getTime();

  setupAS5140();
  // setupIMU();
}

void loop()
{
  if (millis()-sysTimeAtBaseTime >= (secondsUntilNewTime*1000))
  {
    getTime();
  }

  digitalWrite(LED_PIN, LOW);

  // Wait if not connected to wifi
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Connection failed, wait 5 sec...");
    delay(5000);
  }
  else
  { 
    // If connected, upload and blink at specified frequency
    if (int(millis()) - t0 >= (1000 / uploadFrequency))
    {
      t0 = millis();
      Wind windData = prepareWindData();
      protobufBridge.sendWind(windData);
      // Imu imuData = prepareIMUData();
      // protobufBridge.sendIMU(imuData);
      udp_insert.beginPacket(insertServerIP, udpPortRemoteInsert);
      udp_insert.write(protobufBridge.bufferWrapper, protobufBridge.wrapMessageLength);
      udp_insert.endPacket();
    }
    else if (int(millis()) - t0 >= (1000 / (uploadFrequency * 2)))
    {
      digitalWrite(LED_PIN, LOW);
    }
    else
    {
      digitalWrite(LED_PIN, HIGH);
    }
  }
}