#include "init.h"

int64_t newLocalTime()
{
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}

enum pleaseDo
{
  sendRpmHall,
  sendForce,
  sendPower,
  sendImu,
  sendTemperature,
  sendWind,
  controlVesc,
  updateOled
};

void doAtFrequency(pleaseDo whatYouHaveToDo, int &t0, int uploadFrequency, int i=0)
{
  if (int(millis()) - t0 >= (1000 / uploadFrequency))
  {
    switch (whatYouHaveToDo)
    {
#if RPM_HALL
    case sendRpmHall:
    {
      Speed rpmData = hallSensor.prepareData(newLocalTime());
      protobufBridge.sendSpeed(rpmData);
      break;
    }
#endif
#if IMU
    case sendImu:
    {
      Imu imuData = imuSensor.prepareData(newLocalTime());
      protobufBridge.sendIMU(imuData);
      break;
    }
#endif
#if TEMPERATURE
    case sendTemperature:
    {
      Temperature temperatureData = temperatureSensor.prepareData(newLocalTime());
      protobufBridge.sendTemperature(temperatureData);
#endif
#if WIND
      break;
    }
    case sendWind:
    {
      Wind windData = windSensor.prepareData(newLocalTime());
      protobufBridge.sendWind(windData);
      break;
    }
#endif
#if POWER
    case sendPower:
    {
      Power powerData = powerSensor.prepareData(newLocalTime());
      protobufBridge.sendPower(powerData);
      break;
    }
#endif
#if FORCE
    case sendForce:
    {
      Force forceData = forceSensors[i].prepareData(newLocalTime());
      protobufBridge.sendForce(forceData);
      break;
    }
#endif
#if VESC
    case controlVesc:
    {
      vescControl.setRpm();

      Vesc vescData = vescControl.prepareVescData(newLocalTime());
      protobufBridge.sendVesc(vescData);
      udp.beginPacket(insertServerIP, udpPortRemoteInsert);
      udp.write(protobufBridge.bufferWrapper, protobufBridge.wrapMessageLength);
      udp.endPacket();

      Setpoint setpointData = vescControl.prepareSetpointData(newLocalTime());
      protobufBridge.sendSetpoint(setpointData);
      udp.beginPacket(insertServerIP, udpPortRemoteInsert);
      udp.write(protobufBridge.bufferWrapper, protobufBridge.wrapMessageLength);
      udp.endPacket();
      break;
    }
#endif
#if OLED
    case updateOled:
    {
      oled.displayTime(newLocalTime());
      break;
    }
#endif
    default:
      break;
    }
    if (whatYouHaveToDo != controlVesc && whatYouHaveToDo != updateOled)
    {
      udp.beginPacket(insertServerIP, udpPortRemoteInsert);
      udp.write(protobufBridge.bufferWrapper, protobufBridge.wrapMessageLength);
      udp.endPacket();
    }
    t0 = millis();
  }
}

void setup()
{
  Serial.begin(115200); // USB to computer
  Serial.setDebugOutput(true);
  while (!Serial)
  {
    ;
  }

  #if VESC
    vescControl.setup();
  #endif

  #if POWER_DUMP
    powerSensor.PowerDumpSetup();
  #endif
  #if OLED
    oled.setup();
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
    #if OLED
    oled.displayWifi(ssid);
    #endif
  }

  Serial.printf("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

  #if VESC
    server.begin(); // TCP
  // delay(1000);
  // client = server.available();
  #endif

  #if WIND
    windSensor.setupWindDirEncoder();
  #endif
  #if IMU
    imuSensor.setup();
  #endif
  #if RPM_HALL
    hallSensor.setup();
  #endif
  #if FORCE
    for (int i=0; i < (sizeof(forceSensors)/sizeof(*forceSensors)); i++)
    {
      Serial.println(i);
      forceSensors[i].setup();
    }
  #endif

  WiFi.hostByName(addr, insertServerIP); // Define IPAddress object with the ip address string

  // Setup time sync with server att address
  configTzTime("0", addr); // https://github.com/espressif/arduino-esp32/issues/1114 & https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system_time.html

  while (newLocalTime() < 1e6*60*24*365) {
    Serial.print("Get time from "); 
    Serial.println(addr);
    delay(1000);
    #if OLED
    oled.displayIP(addr);
    #endif
  }
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
    #if WIND
    doAtFrequency(sendWind, windSensor.t0, windSensor.uploadFrequency);
    #endif
    #if IMU
    doAtFrequency(sendImu, imuSensor.t0, imuSensor.uploadFrequency);
    #endif
    #if POWER && !POWER_DUMP
    doAtFrequency(sendPower, powerSensor.t0, powerSensor.uploadFrequency);
    #endif
    #if POWER && POWER_DUMP
    doAtFrequency(sendPower, powerSensor.t0, powerSensor.uploadFrequency);
    powerSensor.PowerControl();
    #endif
    #if RPM_HALL
    doAtFrequency(sendRpmHall, hallSensor.t0, hallSensor.uploadFrequency);      
    #endif
    #if TEMPERATURE
    doAtFrequency(sendTemperature, temperatureSensor.t0, temperatureSensor.uploadFrequency);
    #endif
    #if FORCE
    for (int i=0; i < (sizeof(forceSensors)/sizeof(*forceSensors)); i++)
    {
      // Serial.println(i);
      doAtFrequency(sendForce, forceSensors[i].t0, forceSensors[i].uploadFrequency, i);
    }
    #endif
    #if OLED
      doAtFrequency(updateOled, oled.t0, oled.updateFrequency);
    #endif

    #if VESC
    if (!client.connected()) // client = the TCP client who's going to send us something
    {
      client = server.available();
    }
    vescControl.updateRpmSetpoint(client);
    doAtFrequency(controlVesc, vescControl.t0, vescControl.uploadFrequency);
    #endif
  }
  // Serial.printf("time: %lld\n", newLocalTime());
  // delay(1000);
}