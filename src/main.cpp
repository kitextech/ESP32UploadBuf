#include "init.h"

#if IMU
  #include <ImuSensor.h>
  ImuSensor imuSensor(5);
#endif
#if WIND
  #include <WindSensor.h>
  WindSensor windSensor(A0, 2, 0.4, 2, 0.2, 32.4, 3);
#endif
#if POWER
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

void getTime()
{
  Serial.println("I shall now fetch the time!");
  baseTime = timeSync.getTime(timeServerIP, udp);
  sysTimeAtBaseTime = int64_t(millis());
}

int64_t newLocalTime()
{
  return baseTime - sysTimeAtBaseTime + int64_t(millis());
}

#if HAS_VESC
#include <VescStuff.cpp>
#include <HardwareSerial.h>
#include <VescUart.h>

HardwareSerial SerialVesc(2);
VescUart vesc;
int updateFrequencyVesc = 20;
int t0_Vesc = millis();
int uploadFreqVesc = 5;

int tcpPort = 8888;
WiFiServer server(tcpPort);
WiFiClient client = server.available();
uint8_t bufferTCP[128] = {0};

Speed prepareRpmVesc()
{
  Speed rpmData = Speed_init_zero;
  rpmData.time = newLocalTime();
  rpmData.RPM = vesc.data.rpm;
  return rpmData;
}

Power preparePowerVesc()
{
  Power powerData = Power_init_zero;
  powerData.time = newLocalTime();
  powerData.current = vesc.data.avgInputCurrent;
  powerData.voltage = vesc.data.inpVoltage;
  return powerData;
}

void sendVescDataAtFrequency()
{
  if (int(millis()) - t0_Vesc >= (1000 / uploadFreqVesc))
  {
    vesc.getVescValues();
    Speed rpmData = prepareRpmVesc();
    Power powerData = preparePowerVesc();

    protobufBridge.sendSpeed(rpmData);
    udp.beginPacket(insertServerIP, udpPortRemoteInsert);
    udp.write(protobufBridge.bufferWrapper, protobufBridge.wrapMessageLength);
    udp.endPacket();

    protobufBridge.sendPower(powerData);
    udp.beginPacket(insertServerIP, udpPortRemoteInsert);
    udp.write(protobufBridge.bufferWrapper, protobufBridge.wrapMessageLength);
    udp.endPacket();

    t0_Vesc = millis();
  }
}
#endif

enum SendDataType
{
  sendRpmHall,
  sendForce,
  sendPower,
  sendImu,
  sendTemperature,
  sendWind,
  sendVesc
};

void sendDataAtFrequency(SendDataType sendDataType, int &t0, int uploadFrequency)
{
  if (int(millis()) - t0 >= (1000 / uploadFrequency))
  {
    switch (sendDataType)
    {
    case sendRpmHall:
    {
      #if RPM_HALL
        Speed rpmData = hallSensor.prepareData(newLocalTime());
        protobufBridge.sendSpeed(rpmData);
      #endif
      break;
    }
    case sendImu:
    {
      #if IMU
        Imu imuData = imuSensor.prepareData(newLocalTime());
        protobufBridge.sendIMU(imuData);
      #endif
      break;
    }
    case sendTemperature:
    {
      #if TEMPERATURE
      Temperature temperatureData = temperatureSensor.prepareData(newLocalTime());
      protobufBridge.sendTemperature(temperatureData);
      #endif
      break;
    }
    case sendWind:
    {
      #if WIND
        Wind windData = windSensor.prepareData(newLocalTime());
        protobufBridge.sendWind(windData);
      #endif
      break;
    }
    case sendPower:
    {
      #if POWER
        Power powerData = powerSensor.prepareData(newLocalTime());
        protobufBridge.sendPower(powerData);
      #endif
      break;
    }
    // case sendVesc:
    // {
    //   #if HAS_VESC
    //     Speed prepareRpmVesc(newLocalTime());
    //   #endif
    //   break;
    // }
    default:
      break;
    }
    udp.beginPacket(insertServerIP, udpPortRemoteInsert);
    udp.write(protobufBridge.bufferWrapper, protobufBridge.wrapMessageLength);
    udp.endPacket();
    t0 = millis();
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

#if HAS_VESC
void readAndSetRPMByTCP(WiFiClient client)
{
  if (client)
  {
    while (client.connected())
    {
      if (client.available() > 0)
      {
        client.read(bufferTCP, 1);
        String str = String("Message length (bytes): ") + (bufferTCP[0]);
        Serial.println(str);

        int msg_length = bufferTCP[0];

        client.read(bufferTCP, bufferTCP[0]);
        Speed message = Speed_init_zero;
        pb_istream_t stream = pb_istream_from_buffer(bufferTCP, msg_length);
        bool status = pb_decode(&stream, Speed_fields, &message);

        if (!status)
        {
          Serial.printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));
        }
        else
        {
          Serial.printf("Your RPM number was %d!\nSending to the vesc...\n", (int)message.RPM);
          vesc.setRPM(message.RPM);
        }
      }
      return;
    }
  }
}
#endif

void setup()
{
  Serial.begin(115200); // USB to computer
  Serial.setDebugOutput(true);
  while (!Serial)
  {
    ;
  }

  #if HAS_VESC
  SerialVesc.begin(115200, SERIAL_8N1, 16, 17);
  vesc.setSerialPort(&SerialVesc);
  // Serial1.begin(115200);  // rx/tx pins of ESP32 (for the vesc)
  // vesc.setSerialPort(&Serial1);
  #endif

  pinMode(LED_PIN, OUTPUT);

  delay(10);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA); // Necessary?
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.printf("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

  #if HAS_VESC
  server.begin(); // TCP
  // delay(1000);
  // client = server.available();
  #endif

  WiFi.hostByName(addr, insertServerIP); // Define IPAddress object with the ip address string

  // NTC
  // connect to udp_time
  Serial.println("Starting UDP");
  udp.begin(udpPortLocal);
  Serial.print("Local port: ");

  getTime();

  #if WIND
    windSensor.setupWindDirEncoder();
  #endif
  #if IMU
    imuSensor.setup();
  #endif
  #if RPM_HALL
    hallSensor.setup();
  #endif
}

void loop()
{
  digitalWrite(LED_PIN, LOW);

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Connection failed, wait 5 sec...");
    delay(5000);
  }
  else
  {
    if (!(uint32_t(millis()) % (secondsUntilNewTime * 1000)))
    {
      sysTimeAtBaseTime = int64_t(millis());
      getTime();
    }

    #if WIND
      sendDataAtFrequency(sendWind, windSensor.t0, windSensor.uploadFrequency);
    #endif
    #if IMU
      sendDataAtFrequency(sendImu, imuSensor.t0, imuSensor.t0);
    #endif
    #if POWER
      sendDataAtFrequency(sendPower, powerSensor.t0, powerSensor.uploadFrequency);
    #endif
    #if RPM_HALL
      sendDataAtFrequency(sendRpmHall, hallSensor.t0, hallSensor.uploadFrequency);      
    #endif
    #if TEMPERATURE
      sendDataAtFrequency(sendTemperature, temperatureSensor.t0, temperatureSensor.uploadFrequency);
    #endif
    #if HAS_VESC
      if (!client.connected()) // client = the TCP client who's going to send us something
      {
        client = server.available();
      }
      readAndSetRPMByTCP(client);
    #endif
  }
}