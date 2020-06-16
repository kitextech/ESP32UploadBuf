#include "init.h"

// void getTime()
// {
//   Serial.println("I shall now fetch the time!");
//   baseTime = timeSync.getTime(timeServerIP, udp);
//   sysTimeAtBaseTime = int64_t(millis());
// }

int64_t newLocalTime()
{
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}

#if HAS_VESC
Vesc prepareVescData()
{
  Vesc vescData = Vesc_init_zero;
  vescData.time = newLocalTime();
  vescData.avgMotorCurrent = vesc.data.avgMotorCurrent;
  vescData.avgInputCurrent = vesc.data.avgInputCurrent;
  vescData.dutyCycleNow = vesc.data.dutyCycleNow;
  vescData.rpm = vesc.data.rpm;
  vescData.inpVoltage = vesc.data.inpVoltage;
  vescData.ampHours = vesc.data.ampHours;
  vescData.ampHoursCharged = vesc.data.ampHoursCharged;
  vescData.tachometer = vesc.data.tachometer;
  vescData.tachometerAbs = vesc.data.tachometerAbs;
  return vescData;
}

Setpoint prepareSetPoint(float rpm_sp)
{
  Setpoint setpointData Setpoint_init_zero;
  setpointData.time = newLocalTime();
  setpointData.RPM = rpm_sp;
  setpointData.current = 0;
  return setpointData;
}

/*void sendVescDataAtFrequency()
{
  if (int(millis()) - t0_Vesc >= (1000 / uploadFreqVesc))
  {
    vesc.getVescValues();

    float error = vesc.data.rpm - rpmSetpoint;
    pidSUM += error * (1 / uploadFreqVesc);

    if (pidSUM > 10000)
    {
      pidSUM = 10000;
    }
    if (pidSUM < -10000)
    {
      pidSUM = -10000;
    }

    float current_sp = -1 * (0.001 * error + 0.0002 * pidSUM);
    if (current_sp > maxCurrent)
    {
      current_sp = maxCurrent;
    }
    if (current_sp < minCurrent)
    {
      current_sp = minCurrent;
    }

    vesc.setCurrent(current_sp);

    Vesc vescData = prepareVescData();
    protobufBridge.sendVesc(vescData);
    udp.beginPacket(insertServerIP, udpPortRemoteInsert);
    udp.write(protobufBridge.bufferWrapper, protobufBridge.wrapMessageLength);
    udp.endPacket();

    Setpoint setpointData = prepareSetPoint(current_sp);
    protobufBridge.sendSetpoint(setpointData);
    udp.beginPacket(insertServerIP, udpPortRemoteInsert);
    udp.write(protobufBridge.bufferWrapper, protobufBridge.wrapMessageLength);
    udp.endPacket();
    
    t0_Vesc = millis();
  }
}*/

int mode(int a[],int n) {
   int maxValue = 0, maxCount = 0, i, j;

   for (i = 0; i < n; ++i) {
      int count = 0;
      
      for (j = 0; j < n; ++j) {
         if (a[j] == a[i])
         ++count;
      }
      
      if (count > maxCount) {
         maxCount = count;
         maxValue = a[i];
      }
   }

   return maxValue;
}

void updateArray(int newElement, int n)
{
  for (int i = n-1; i > 0; --i) {
    rpmSetpointArray[i] = rpmSetpointArray[i-1] ;
  } 
  rpmSetpointArray[0] = newElement;
}

void readAndSetRPMByTCP(WiFiClient client)
{
  if (client)
  {
    while (client.connected())
    {
      if (client.available() > 0)
      {
        client.read(bufferTCP, 1);
        // String str = String("Message length (bytes): ") + (bufferTCP[0]);
        // Serial.println(str);

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
          updateArray((int)(message.RPM), MODE_ARRAY_LENGTH);
          if (message.RPM == mode(rpmSetpointArray, MODE_ARRAY_LENGTH) && message.RPM != rpmSetpoint)
          {
            Serial.printf("RPM is set to %d\n", (int)message.RPM);  
            rpmDiff = message.RPM - (float)rpmSetpoint;
            t0_ramp = millis();
            rpmSetpoint = (float)mode(rpmSetpointArray, MODE_ARRAY_LENGTH);
          }
        }
      }
      return;
    }
  }
}

void setRPMByTCP()
{
  if (int(millis()) - t0_Vesc >= (1000 / uploadFreqVesc))
  {
    vesc.getVescValues();
    if (t0_ramp + rampingTime > millis())
      rpm_sp = (float(millis()) - float(t0_ramp)) / rampingTime * rpmDiff + ((float)rpmSetpoint - rpmDiff);
    else
      rpm_sp = rpmSetpoint;
    Serial.printf("RPM: %f\n", rpm_sp);  
    vesc.setRPM(rpm_sp);

    Vesc vescData = prepareVescData();
    protobufBridge.sendVesc(vescData);
    udp.beginPacket(insertServerIP, udpPortRemoteInsert);
    udp.write(protobufBridge.bufferWrapper, protobufBridge.wrapMessageLength);
    udp.endPacket();

    Setpoint setpointData = prepareSetPoint(rpm_sp);
    protobufBridge.sendSetpoint(setpointData);
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
  sendVesc,
  sendOled
};

void sendDataAtFrequency(SendDataType sendDataType, int &t0, int uploadFrequency, int i=0)
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
    case sendForce:
    {
#if FORCE
      Force forceData = forceSensors[i].prepareData(newLocalTime());
      protobufBridge.sendForce(forceData);
#endif
    break;
    }
    case sendOled:
    {
#if OLED
      oled.displayTime(newLocalTime());
#endif
    break;
    }
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

#if POWER_DUMP
  powerSensor.PowerDumpSetup();
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
  // Serial.println("Starting UDP");
  // udp.begin(udpPortLocal);
  // Serial.print("Local port: ");


  configTzTime("0", addr); // https://github.com/espressif/arduino-esp32/issues/1114 & https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system_time.html

  while (newLocalTime() < 1e6*60*24*365) {
    Serial.print("Get time from "); Serial.println(addr);
    delay(1000);
  }
 


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
#if OLED
  oled.setup();
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
    #if WIND
    sendDataAtFrequency(sendWind, windSensor.t0, windSensor.uploadFrequency);
    #endif
    #if IMU
    sendDataAtFrequency(sendImu, imuSensor.t0, imuSensor.t0);
    #endif
    #if POWER && !POWER_DUMP
    sendDataAtFrequency(sendPower, powerSensor.t0, powerSensor.uploadFrequency);
    #endif
    #if POWER && POWER_DUMP
    sendDataAtFrequency(sendPower, powerSensor.t0, powerSensor.uploadFrequency);
    powerSensor.PowerControl();
    #endif
    #if RPM_HALL
    sendDataAtFrequency(sendRpmHall, hallSensor.t0, hallSensor.uploadFrequency);      
    #endif
    #if TEMPERATURE
    sendDataAtFrequency(sendTemperature, temperatureSensor.t0, temperatureSensor.uploadFrequency);
    #endif
    #if FORCE
    for (int i=0; i < (sizeof(forceSensors)/sizeof(*forceSensors)); i++)
    {
      // Serial.println(i);
      sendDataAtFrequency(sendForce, forceSensors[i].t0, forceSensors[i].uploadFrequency, i);
    }
    #endif
    #if OLED
      sendDataAtFrequency(sendOled, oled.t0, oled.updateFrequency);
    #endif

    #if HAS_VESC
    if (!client.connected()) // client = the TCP client who's going to send us something
    {
      client = server.available();
    }
    readAndSetRPMByTCP(client);
    // sendVescDataAtFrequency();
    setRPMByTCP();
    #endif
  }
  // Serial.printf("time: %lld\n", newLocalTime());
  // delay(1000);
}