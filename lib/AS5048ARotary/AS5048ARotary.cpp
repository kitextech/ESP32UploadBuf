#include "AS5048ARotary.h"

AS5048ARotary::AS5048ARotary() {
  Serial.println("Created an AHT20 Humidity sensor handling object");
}

void AS5048ARotary::setup(ProtobufBridge bridge)
{ // BNO-055 SETUP
  protobridge = bridge;
  Serial.println("Setup AS5048ARotary");
	angleSensor.begin();
}


void AS5048ARotary::loopWifiAndTime(int64_t time) {
    if( uploadTimer.doRun() ) {
    // prepareData(time);
    protobridge.sendWindDirection(prepareData(time));
  }
}


WindDirection AS5048ARotary::prepareData(int64_t time)
{ 
  
  float val = angleSensor.getRotationInDegrees();
	// Serial.print("\nGot rotation of: ");

  float offset = 124.10;
  val -= offset;
  if (val < 0) {
    val += 360;
  }

	// Serial.println(val);

	// Serial.print("State: ");
	// angleSensor.printState();
	// Serial.print("Errors: ");
	// Serial.println(angleSensor.getErrors());
  
  WindDirection windDirData = WindDirection_init_zero;
  windDirData.time = time;
  windDirData.direction = val;
 
  return windDirData;
}