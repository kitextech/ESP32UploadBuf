#include "Util.h"

void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.print("Day of week: ");
  Serial.println(&timeinfo, "%A");
  Serial.print("Month: ");
  Serial.println(&timeinfo, "%B");
  Serial.print("Day of Month: ");
  Serial.println(&timeinfo, "%d");
  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");

  Serial.println("Time variables");
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  Serial.println(timeHour);
  char timeWeekDay[10];
  strftime(timeWeekDay,10, "%A", &timeinfo);
  Serial.println(timeWeekDay);
  Serial.println();
}

//**********************************
//************ SETUP OTA ****************
//**********************************

bool setupOTA(const char *hostname)
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
  return true;
}


//**********************************
//************ ReconnectoToWifi ****************
//**********************************

void WifiReconnect::reconnectToWifi() {

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


boolean WifiReconnect::checkWifi(const char *ssid) {
  
  bool reportStatus = wifiReport.doRun();

  if (WiFi.status() != WL_CONNECTED) {
    if (wifiReconnect.doRun()) {
      reconnectToWifi();
    }

    if (reportStatus) {
      Serial.println("not connected to WIFI");
    }
    return false;
  }
  if (reportStatus) {
      Serial.print("Connected to WIFI: ");
      Serial.println(ssid);
  }

  return true;
}