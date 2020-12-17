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
  updateOled,
  sendBlade
};

//**********************************
//************ DoAtFrequency ****************
//**********************************

void doAtFrequency(pleaseDo whatYouHaveToDo, int &t0, int uploadFrequency, int i = 0)
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
#if TEMPERATURE
    case sendTemperature:
    {
      Temperature temperatureData = temperatureSensor.prepareData(newLocalTime());
      protobufBridge.sendTemperature(temperatureData);
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
#if OLED
    case updateOled:
    {
      oled.displayTime(newLocalTime());
      break;
    }
#endif
#if BLADE
    case sendBlade:
    {
      BladeControl bladeControl = bladePitchControl.prepareData(newLocalTime());
      protobufBridge.sendBladeControl(bladeControl);
      break;
    }


#endif

    default:
      break;
    }
    if (whatYouHaveToDo != controlVesc && whatYouHaveToDo != updateOled)
    {
      // Serial.println("hello");
      
      udp.beginPacket(insertServerIP, udpPortRemoteInsert);
      udp.write(protobufBridge.bufferWrapper, protobufBridge.wrapMessageLength);
      udp.endPacket();
    }
    t0 = millis();
  }
}

//**********************************
//************ SETUP ****************
//**********************************

void setup()
{
  Serial.begin(115200); // USB to computer
  Serial.setDebugOutput(true);
  while (!Serial) // wait for serial to be active
  {
    ;
  }
  pinMode(LED_PIN, OUTPUT); // really?

  delay(10);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Listen for UDP packages
  Serial.print("Listen for UDP packages on port: "); // can be done even without wifi connection
  Serial.println(udpPortLocalRecieve);
  udp.begin(udpPortLocalRecieve);

  // configure protobridge 
  WiFi.hostByName(addr, insertServerIP); // Define IPAddress object with the ip address string
  // = ProtobufBridge(udp, insertServerIP, udpPortRemoteInsert);
  protobufBridge.udp = &udp;
  protobufBridge.serverip = insertServerIP;
  protobufBridge.serverport = udpPortRemoteInsert;
  // Setup time sync with server att address
  configTzTime("0", addr); // https://github.com/espressif/arduino-esp32/issues/1114 & https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system_time.html

#if VESC
  vescControl.setup(protobufBridge);
#endif

#if POWER_DUMP
  powerSensor.PowerDumpSetup();
#endif
#if OLED
  oled.setup();
#endif



#if WIND  
  windSensor.setup(protobufBridge);
#endif
#if IMU
  imuSensor.setup(protobufBridge);
#endif
#if ACC
  accSensor.setup();
#endif
#if RPM_HALL
  hallSensor.setup();
#endif
#if FORCE
  for (int i = 0; i < (sizeof(forceSensors) / sizeof(*forceSensors)); i++)
  {
    Serial.println(i);
    forceSensors[i].setup(protobufBridge);
  }
#endif

#if BLADE
  bladePitchControl.setup();
  
#endif
  
}


//**********************************
//************ SETUP OTA ****************
//**********************************

void setupOTA()
{
    // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(hostname);

  // No authentication by default
  ArduinoOTA.setPassword("morepower");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Hostname: ");
  Serial.print(ArduinoOTA.getHostname());
  Serial.println(".local");
  OTASetup = true;
}

//**********************************
//************ ReconnectoToWifi ****************
//**********************************

void reconnectToWifi() {

    // wifi down, reconnect here
  if (wifiReconnectCount == 10) {
    switch (WiFi.status())
      {
      case WL_IDLE_STATUS:
        Serial.printf("WL_IDLE_STATUS");
        break;
      case WL_NO_SSID_AVAIL:
        Serial.printf("WL_NO_SSID_AVAIL");
        break;
      case WL_SCAN_COMPLETED:
        Serial.printf("WL_SCAN_COMPLETED");
        break;
      case WL_CONNECTED:
        Serial.printf("WL_CONNECTED");
        break;
      case WL_CONNECT_FAILED:
        Serial.printf("WL_CONNECT_FAILED");
        break;
      case WL_DISCONNECTED:
        Serial.printf("WL_DISCONNECTED");
        break;
      
      default:
        break;
      }
    WiFi.begin();
    Serial.println("Call to Wifi Begin");
    wifiReconnectCount = 0;
  }
  wifiReconnectCount++;
}


//**********************************
//************ Check Wifi ****************
//**********************************


boolean checkWifi(bool reportStatus) {
  
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiReconnect.doRun()) {
      reconnectToWifi();
    }

    if (reportStatus) {
      Serial.println("not connected to WIFI");
    }
    return false;
  }
  return true;
}

//**********************************
//************ Check Time ****************
//**********************************

boolean checkTime(bool reportStatus) {
  
  if (newLocalTime() < 1e6 * 60 * 24 * 365)
  { 
    if (reportStatus) {
      Serial.print("Get time from ");
      Serial.print(addr);
      Serial.println(" ...");
    }
    return false;
  }
  return true;
}


//**********************************
//************ Wifi Loop ****************
//**********************************

void wifiLoop() {
  if (!OTASetup) {
    setupOTA();
  }
  ArduinoOTA.handle();
}

//**********************************
//************ Wifi And Time loop ****************
//**********************************

void wifiAndTimeLoop() {


  
#if WIND
    windSensor.loopWifiAndTime(newLocalTime());
#endif

#if IMU
    imuSensor.loopWifiAndTime(newLocalTime());  
    // doAtFrequency(sendImu, imuSensor.t0, imuSensor.uploadFrequency);
#endif

#if ACC 
  if (accSensor.doUpload.doRun() ) {
      Imu imuData = accSensor.prepareData(newLocalTime()); // the Accelerometer can also send partial IMU data
      protobufBridge.sendIMU(imuData);

      udp.beginPacket(insertServerIP, udpPortRemoteInsert); // refactor later 
      udp.write(protobufBridge.bufferWrapper, protobufBridge.wrapMessageLength);
      udp.endPacket();
  }

#endif

#if POWER && !POWER_DUMP
    doAtFrequency(sendPower, powerSensor.t0, powerSensor.uploadFrequency);
#endif

#if POWER && POWER_DUMP
    doAtFrequency(sendPower, powerSensor.t0, powerSensor.uploadFrequency);
#endif

#if RPM_HALL
    doAtFrequency(sendRpmHall, hallSensor.t0, hallSensor.uploadFrequency);
#endif

#if TEMPERATURE
    doAtFrequency(sendTemperature, temperatureSensor.t0, temperatureSensor.uploadFrequency);
#endif

#if FORCE
    for (int i = 0; i < (sizeof(forceSensors) / sizeof(*forceSensors)); i++)
  {
    forceSensors[i].loopWifiAndTime(newLocalTime()); 
  }
#endif

#if OLED
    doAtFrequency(updateOled, oled.t0, oled.updateFrequency);
#endif

#if VESC

  vescControl.loopWifiAndTime(newLocalTime());

  // check the udp port for new turbine control settings
  udp.parsePacket();
  int n = udp.read(UDPInBuffer, 128);
  if (n > 0) {
    vescControl.updateTurbineControl(UDPInBuffer, n);
  } 

#endif

#if BLADE
    //bladePitchControl.loop(udp, UDPInBuffer);

  udp.parsePacket();
  int n = udp.read(UDPInBuffer, 128);

  if (n > 0) {
    bladePitchControl.loop(UDPInBuffer, n);
  } 
  doAtFrequency(sendBlade, bladePitchControl.t0 , bladePitchControl.uploadFrequency);
  // else {
  //   Serial.print(".");
  //   delay(100);
  // }

#endif

}

//**********************************
//************ No Wifi and Time Loop ****************
//**********************************

void noWifiAndTimeLoop() {

#if POWER && POWER_DUMP
  if (powerSensor.noWifiLoopTimer.doRun()) {
    powerSensor.readVoltageCurrent();
    powerSensor.PowerControl();
    powerSensor.Indicator();
    powerSensor.chargeOnOff();
  }
#endif

#if VESC
  vescControl.loop();
#endif

#if WIND
  windSensor.loop();
#endif

}

//**********************************
//************ LOOP ****************
//**********************************
void loop()
{ 
  
  if ( checkWifi( wifiTimeReport.doRun() ) ) {
    wifiLoop();

    if (checkTime( wifiTimeReport.doRun()  )) {
      
      
      wifiAndTimeLoop();
    }
  }

  noWifiAndTimeLoop();
}

      // unsigned long start = millis();
      // Imu imuData = imuSensor.prepareData(newLocalTime());
      // unsigned long next = millis();
      // protobufBridge.sendIMU(imuData);
      // unsigned long end = millis();
      // Serial.println(end-next, next-start);