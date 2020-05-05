
/*
RPM part is based on Arduino Hall Effect Sensor Project by Arvind Sanjeev
Link: http://diyhacking.com
Temperature part is based on Adafruit Learning System guide on Thermistors by Limor Fried, Adafruit Industries
Link:  https://learn.adafruit.com/thermistor/overview

Both modified by Bertalan Kovács (bertalan@kitex.tech)
Install ESP8266 on Arduino IDE: https://github.com/esp8266/Arduino/blob/master/README.md
*/

#include <Arduino.h>
// #include "msg.pb.h"
// #include "schema.pb.h"
// #include "pb_common.h"
// #include "pb.h"
// #include "pb_encode.h"

#include<iostream>


// #include <WiFi.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h> // NTC
#include <TimeSync.h>
#include <Adafruit_Sensor.h>  // BNO-055
#include <Adafruit_BNO055.h>  // BNO-055
#include <ProtobufBridge.h>

#include <MedianFilter.h>

/*
 * AS5040.h          -> library for reading data through SSI protocol from AS5140 magnetic sensor
 * MedianFilter.h    -> library for filtering the noisy recieved analog data
 * ESP8266WiFi.h     -> library for handling modularity of wifi on ESP8266 
 * WiFiUdp.h         -> library for reading time by UDP protocol from NTP server
 */


// enum Sensor {Imu, Wind} sensor;
// sensor = Imu;

// enum direction {East, West, North, South}dir;
// dir = East;

const char* ssid     = "kitex";
const char* password = "morepower";

uint8_t buffer[128];
size_t imuMessageLength;
size_t wrapMessageLength;
uint8_t bufferWrapper[512];

WiFiClient client;
ProtobufBridge protobufBridge;

//const char* addr     = "192.168.8.144"; // Local IP of the black-pearl pi
const char* addr     = "192.168.8.126"; // Local IP of the okholms Laptop

const uint16_t port  = 10101;

// NTC
IPAddress timeServerIP;
WiFiUDP udp;
unsigned int localPort = 2390;

// Time
TimeSync timeSync;
int64_t baseTime;
int64_t sysTimeAtBaseTime;

int uploadFrequencyIMU = 50;
int uploadFrequencyWind = 2;
int uploadFrequencyMotor = 1;
int t0_IMU = millis();
int t0_Motor = millis();

// BNO-055
#define BNO055_SAMPLERATE_DELAY_MS (10)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

// Wind speed (analog read)
const int AnalogPin = A0;
const int VoltdivRatio = 2;
const float minVoltage = 0.4 / VoltdivRatio;
const float maxVoltage = 2.0 / VoltdivRatio;
const float minSpeed = 0.2;
const float maxSpeed = 32.4;
const int updateFreq = 5;
MedianFilter MedFilter(10, 0);

//// Motor measurements (RPM + temperature)
// Hall sensor settings
#define HALL 14             // D5 pin on ESP8266 NodeMCU for one hall sensor connection
#define POLE_PAIR_NUM 7     // 14 poles -> 7 pole pairs, counted manually

// Teperature settings
#define THERMISTORPIN A0    // which analog pin to connect
#define THERMISTORNOMINAL 10000 // resistance at 25 degrees C
#define TEMPERATURENOMINAL 25 // temp. for nominal resistance (almost always 25 C)
#define NUMSAMPLES 5        // how many samples to take and average, more takes longer but is more 'smooth'
#define BCOEFFICIENT 3950   // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 10000  // the value of the 'other' resistor

unsigned int detection = 0;     // Detection counter by the hall sensor
float rpm = 0;

int adc_samples[NUMSAMPLES];

// Define the LED pin - different for different ESP's
// #define LED_PIN LED_BUILTIN // For normal Arduino, possibly other ESP's
#define LED_PIN 0

ICACHE_RAM_ATTR void magnet_detect()  // This function is called whenever a magnet/interrupt is detected
{
   detection++;
}

float mapFloat(float value, float in_min, float in_max, float out_min, float out_max) {
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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

int64_t getNewTime() {
  return baseTime - sysTimeAtBaseTime + int64_t(millis());
}

void setupMotor()
{
   pinMode(HALL, INPUT_PULLUP);       // Pulling up the pin (equivalent of using the 10kOhm resistance on the board)
   attachInterrupt(digitalPinToInterrupt(HALL), magnet_detect, RISING); // Initialize the intterrupt pin
}

Imu prepareIMUData() {
  Imu imuData = Imu_init_zero;
  
  imuData.time = getNewTime();

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

Wind prepareWindData() {
  Wind windData = Wind_init_zero;

  windData.time = getNewTime();

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
  windData.direction = 0.0f;

  return windData;
}

Speed prepareRPMData() {
  Speed rpmData = Speed_init_zero;

  rpmData.time = getNewTime();
  // 60 is to convert rps to rpm; 1000 to corrigate ms to s; 
  // division by pole pairs is to get the whole rotation not only between two plus polarity magnet ¨
  rpm = float(60*1000) /( float(millis() - t0_Motor)) * float(detection) / float(POLE_PAIR_NUM);
  detection = 0;
  // Serial.print("RPM: "); 
  // Serial.println(rpm);


  rpmData.RPM = rpm;

  return rpmData;
}

Temperature prepareTemperatureData() {
  Temperature temperatureData = Temperature_init_zero;

  temperatureData.time = getNewTime();

  uint8_t i;
  float average;
  
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
    adc_samples[i] = analogRead(THERMISTORPIN);
  }
  
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
      average += adc_samples[i];
  }
  average /= NUMSAMPLES;
  
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
    
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  
  // Serial.print("Temperature: "); 
  // Serial.print(steinhart);
  // Serial.println(" °C");

  temperatureData.temperature = steinhart;

  return temperatureData;
}

void setup() {
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
  //Serial.println(up);

  Serial.println("I shall now fetch the time!");
  timeServerIP = IPAddress();
  timeServerIP.fromString(addr);
  baseTime = timeSync.getTime(timeServerIP, udp);
  sysTimeAtBaseTime = int64_t(millis());

  // setupIMU();
  setupMotor();
}


void loop() {
  digitalWrite(LED_PIN, LOW);

  // Wait if not connected to wifi
  if (!client.connected()) {
    client.connect(addr, port);
    Serial.println("connection failed");
    Serial.println("wait 5 sec to reconnect...");
    delay(5000); // Add error blinking here
  } 
  else {
    // If connected, upload and blink at specified frequency
    if (int(millis())-t0_IMU >= (1000/uploadFrequencyIMU)) {
      t0_IMU = millis();
      Wind windData = prepareWindData();
      protobufBridge.sendWind(windData);
      Imu imuData = prepareIMUData();
      protobufBridge.sendIMU(imuData);
      client.write(protobufBridge.bufferWrapper, protobufBridge.wrapMessageLength);
    } 
    else if (int(millis())-t0_IMU >= (1000/(uploadFrequencyIMU*2))) {
      digitalWrite(LED_PIN, LOW);
      // Serial.println("LOW");
    } else {
      digitalWrite(LED_PIN, HIGH);
      // Serial.println("HIGH");
    }

    if (int(millis())-t0_Motor >= (1000/(uploadFrequencyMotor))) {
      Speed rpmData = prepareRPMData();
      protobufBridge.sendSpeed(rpmData);
      client.write(protobufBridge.bufferWrapper, protobufBridge.wrapMessageLength);

      Temperature temperatureData = prepareTemperatureData();
      protobufBridge.sendTemperature(temperatureData);
      client.write(protobufBridge.bufferWrapper, protobufBridge.wrapMessageLength);
      t0_Motor = millis();
    }
  }
}