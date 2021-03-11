#include "init.h"

// can't move to util for some reason!
int64_t newLocalTime(){
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
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
  // configTzTime("0", addr, "0.dk.pool.ntp.org", "1.dk.pool.ntp.org"); // https://github.com/espressif/arduino-esp32/issues/1114 & https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system_time.html
  configTime(0, 0, NTPServer); // equivalent to using configTzTime. Using backup NTP servers doesn't seem effective. One can manually change them with success. 


#if VESC
  vescControl.setup(protobufBridge);
#endif
#if POWER_DUMP
  powerSensor.PowerDumpSetup();
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
#if HUMTEMP
  ahtSensor.setup(protobufBridge);
#endif
#if WINDDIRECTION
  rotarySensor.setup(protobufBridge); // update delay
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
  bladePitchControl.setup(protobufBridge);
#endif
}


//**********************************
//************ Wifi Loop ****************
//**********************************

void wifiLoop() {
  if (!OTASetup) {
    OTASetup = setupOTA(hostname);
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

#if HUMTEMP 
  ahtSensor.loopWifiAndTime(newLocalTime());
#endif

#if WINDDIRECTION
  rotarySensor.loopWifiAndTime(newLocalTime()); // update delay
#endif

#if POWER && !POWER_DUMP
    doAtFrequency(sendPower, powerSensor.t0, powerSensor.uploadFrequency);
#endif

#if POWER && POWER_DUMP
    doAtFrequency(sendPower, powerSensor.t0, powerSensor.uploadFrequency);
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
  udp.parsePacket();
  int n = udp.read(UDPInBuffer, 128);

  if (n > 0) {
    bladePitchControl.loop(UDPInBuffer, n);
  } 

  bladePitchControl.loopWifiAndTime(newLocalTime());

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

// #if WINDDIRECTION
//   rotarySensor.loopWifiAndTime(0); // HACK
// #endif

#if TEST
  if (timer.doRun()) {
    // if (!checkTime(false) && millis() > 20e3) {
    //   configTime(0, 0, NTPServer);
    //   Serial.println("Changed to fallback timeserver");
    // }
    //printLocalTime();
  }
#endif
  delay(10);
}

//**********************************
//************ LOOP ****************
//**********************************
void loop()
{ 
  if ( wifiReconnect.checkWifi( ssid ) ) {
    wifiLoop();

    if (checkTime( newLocalTime(), timeReport.doRun(), NTPServer)) {
      wifiAndTimeLoop();
    }
  }

  noWifiAndTimeLoop();
}