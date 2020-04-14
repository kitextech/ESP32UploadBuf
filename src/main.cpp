#include <Arduino.h>
#include "msg.pb.h"
#include "schema.pb.h"
#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"

#include <WiFi.h>
#include <WiFiUdp.h> // NTC


const char* ssid     = "kitex";
const char* password = "morepower";

uint8_t buffer[128];
size_t imuMessageLength;
size_t wrapMessageLength;
uint8_t bufferWrapper[512];

WiFiClient client;

const char* addr     = "192.168.8.126";
const uint16_t port  = 1337;



// NTC

struct parsedTime {
  int hour;
  int minute;
  int second;
  uint16_t milisecond;
};

struct parsedTime Ptime;

IPAddress timeServerIP;
WiFiUDP udp;

const int   NTP_PACKET_SIZE = 48;
const char* ntpServerName = "dk.pool.ntp.org"; //"time.nist.gov";
byte        packetBuffer[ NTP_PACKET_SIZE];
unsigned int localPort = 2390; 

int64_t baseTime;
int64_t sysTimeAtBaseTime;


// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  // Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

int64_t Parse(struct parsedTime *timeStruct, byte* packet){
   // make the POSIX time
   unsigned long highWord = word(packet[40], packet[41]);
   unsigned long lowWord  = word(packet[42], packet[43]);
   unsigned long secsSince1900 = highWord << 16 | lowWord;
   // convert to NTP time
   const unsigned long seventyYears = 2208988800UL; 
   unsigned long epoch = secsSince1900 - seventyYears;
   // get the hour,minute,seconds
    timeStruct->hour   = (epoch % 86400L) / 3600;         
    timeStruct->minute = (epoch % 3600) / 60;
    timeStruct->second = (epoch % 60);
    // get the fraction of seconds 
    // https://arduino.stackexchange.com/questions/49567/synching-local-clock-usign-ntp-to-milliseconds
    uint32_t frac  = (uint32_t) packetBuffer[44] << 24
                   | (uint32_t) packetBuffer[45] << 16
                   | (uint32_t) packetBuffer[46] <<  8
                   | (uint32_t) packetBuffer[47] <<  0;
    timeStruct->milisecond = ((uint64_t) frac * 1000 ) >> 32;


    return int64_t( int64_t(epoch) * int64_t(1000) + int64_t( timeStruct->milisecond ) );
}


int64_t getTime() {
  WiFi.hostByName(ntpServerName, timeServerIP);
  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  delay(500);
  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("no packet yet");
  }
  else {
   udp.read(packetBuffer, NTP_PACKET_SIZE);
  int64_t time = Parse(&Ptime, packetBuffer);
  // print in serial port 
  Serial.print (Ptime.hour);
  Serial.print (":");
  Serial.print (Ptime.minute);
  Serial.print (":");
  Serial.print (Ptime.second);
  Serial.print (":");
  Serial.println (Ptime.milisecond);   
  
  return time;
  }
  Serial.println ( "PROBLEM!");   
  return 0;
}

// WebSocketsClient webSocket;

// void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
// 	const uint8_t* src = (const uint8_t*) mem;
// 	Serial.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
// 	for(uint32_t i = 0; i < len; i++) {
// 		if(i % cols == 0) {
// 			Serial.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
// 		}
// 		Serial.printf("%02X ", *src);
// 		src++;
// 	}
// 	Serial.printf("\n");
// }

// void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

// 	switch(type) {
// 		case WStype_DISCONNECTED:
// 			Serial.printf("[WSc] Disconnected!\n");
// 			break;
// 		case WStype_CONNECTED:
// 			Serial.printf("[WSc] Connected to url: %s\n", payload);

// 			// send message to server when Connected
// 			webSocket.sendTXT("Connected");
// 			break;
// 		case WStype_TEXT:
// 			Serial.printf("[WSc] get text: %s\n", payload);

// 			// send message to server
// 			// webSocket.sendTXT("message here");
// 			break;
// 		case WStype_BIN:
// 			Serial.printf("[WSc] get binary length: %u\n", length);
// 			hexdump(payload, length);

// 			// send data to server
// 			// webSocket.sendBIN(payload, length);
// 			break;
// 		case WStype_ERROR:
//       Serial.println("some Error");
// 		case WStype_FRAGMENT_TEXT_START:
// 		case WStype_FRAGMENT_BIN_START:
// 		case WStype_FRAGMENT:
// 		case WStype_FRAGMENT_FIN:
// 			break;
// 	}
// }

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


  // // server address, port and URL
	// webSocket.begin("192.168.8.126", 81, "/");

	// // event handler
	// webSocket.onEvent(webSocketEvent);

	// // use HTTP Basic Authorization this is optional remove if not needed
	// webSocket.setAuthorization("user", "Password");

	// // try ever 5000 again if connection has failed
	// webSocket.setReconnectInterval(5000);


  // NTC
   // connect to udp 
    Serial.println("Starting UDP");
    udp.begin(localPort);
    Serial.print("Local port: ");
    // Serial.println(up);

    baseTime = getTime();
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