#include <Arduino.h>
// #include "msg.pb.h"
#include "schema.pb.h"
#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"

// #include <WiFi.h>
// #include <WiFi.h>
// #include <WiFiUdp.h> // NTC

#include <TimeSync.h>


const char* ssid     = "kitex";
const char* password = "morepower";

uint8_t buffer[128];
size_t imuMessageLength;
size_t wrapMessageLength;
uint8_t bufferWrapper[512];

WiFiClient client;

TimeSync timeSync;

const char* addr     = "192.168.8.107";
const uint16_t port  = 10101;



// NTC

IPAddress timeServerIP;
// WiFiUDP udp;

unsigned int localPort = 2390;

int64_t baseTime;
int64_t sysTimeAtBaseTime;

void setup() {
  // put your setup code here, to run once:
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
  timeSync.udp.begin(localPort);
  Serial.print("Local port: ");
  // Serial.println(up);

  baseTime = timeSync.getTime(timeServerIP);
  sysTimeAtBaseTime = int64_t(millis());

  // delay(100000);
}


int64_t getNewTime() {
  return baseTime - sysTimeAtBaseTime + int64_t(millis());
}

bool write_imuBuffer(pb_ostream_t *stream, const pb_field_iter_t *field, void * const *arg)
{
    //char *str = "Hello world!";
    
    if (!pb_encode_tag_for_field(stream, field))
        return false;
    
    return pb_encode_string(stream, (uint8_t*) &buffer, imuMessageLength);
}

float random() {
  return float(random(0, 100))/100.0;
}

void prepareIMUData() {
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  Quaternion orientation = Quaternion_init_zero; // or {1,0,0,1}
  orientation.x = random();
  orientation.y = random();
  orientation.z = random();
  orientation.w = random();

  Vector3 acc = {random(), random(), random()};
  Vector3 gyro = {random(), random(), random()};

  Imu imu = Imu_init_zero;
  imu.has_acc = true;
  imu.acc = acc;
  imu.has_gyro = true;
  imu.gyro = gyro;
  imu.has_orientation = true;
  imu.orientation = orientation;
  imu.time = getNewTime();

  // write IMU data
  bool status = pb_encode(&stream, Imu_fields, &imu);
 
  if (!status)
  {
      Serial.println("Failed to encode imu");
      return;
  }
 
  // Serial.print("Message Length imu: ");
  // Serial.println(stream.bytes_written);
  imuMessageLength = stream.bytes_written;
  

  stream = pb_ostream_from_buffer(bufferWrapper, sizeof(bufferWrapper));

  // wrapper
  Wrapper wrap = Wrapper_init_zero;
  wrap.type = Wrapper_DataType_IMU;
  wrap.data.funcs.encode = &write_imuBuffer;

  status = pb_encode(&stream, Wrapper_fields, &wrap);
 
  if (!status)
  {
      Serial.println("Failed to encode wrapper");
      return;
  }

  wrapMessageLength = stream.bytes_written;

  // Serial.print("Message Length wrapper: ");
  Serial.println(stream.bytes_written);
 
  // Serial.print("Message wrapper: ");
 
  // for(int i = 0; i<stream.bytes_written; i++){
  //   Serial.printf("%02X",bufferWrapper[i]);
  // }

  // Serial.println();
}


void loop() {
  // put your main code here, to run repeatedly:

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

  prepareIMUData();

  client.write(bufferWrapper, wrapMessageLength);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(20);
  // Serial.println(".");

  // prepareIMUData();
  // webSocket.sendBIN( bufferWrapper, wrapMessageLength);
  
}