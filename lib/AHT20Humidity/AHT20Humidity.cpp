#include "AHT20Humidity.h"

AHT20Humidity::AHT20Humidity() {
  Serial.println("Created an AHT20 Humidity sensor handling object");
}

void AHT20Humidity::setup(ProtobufBridge bridge)
{ // BNO-055 SETUP
  protobridge = bridge;
  Serial.println("Adafruit AHT20 test!");
  Serial.println("");
  if (!aht.begin())
  {
    /* There was a problem detecting the AHT20 ... check your connections */
    Serial.println("Couldnt start");
  }
  delay(100);
  Serial.println("AHT20 found!");
}


void AHT20Humidity::loopWifiAndTime(int64_t time) {
    if( uploadTimer.doRun() ) {
    protobridge.sendHumidityTemperature(prepareData(time));
  }
}


HumidityTemperature AHT20Humidity::prepareData(int64_t time)
{ 
  HumidityTemperature humTempData = HumidityTemperature_init_zero;
  humTempData.time = time;

    /* Get a new sensor event */ 
    sensors_event_t humidityEvent, tempEvent;
  aht.getEvent(&humidityEvent, &tempEvent);// populate temp and humidity objects with fresh data

  humTempData.humidity = humidityEvent.relative_humidity;
  humTempData.temperature = tempEvent.temperature;
  
  return humTempData;
}