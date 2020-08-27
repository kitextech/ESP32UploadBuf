#include "AccSensor.h"

AccSensor::AccSensor(uint16_t updatePeriod) :
doUpload(updatePeriod)
{
  Serial.println("Created an IMU sensor handling object");
}

void AccSensor::setup()
{ // BNO-055 SETUP
  Serial.println("Adafruit MMA8451 test!");
  Serial.println("");
  if (!mma.begin())
  {
    /* There was a problem detecting the MMA8451 ... check your connections */
    Serial.println("Couldnt start");
  }
  delay(100);
  Serial.println("MMA8451 found!");


  mma.setRange(MMA8451_RANGE_4_G);
  
  Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  Serial.println("G");
}

Imu AccSensor::prepareData(int64_t time)
{ 
  Imu imuData = Imu_init_zero;
  imuData.time = time;

    /* Get a new sensor event */ 
  sensors_event_t event; 
  mma.getEvent(&event);

  imuData.has_acc = true;
  imuData.acc.x = event.acceleration.x;
  imuData.acc.y = event.acceleration.y;
  imuData.acc.z = event.acceleration.z;

  imuData.has_gyro = false;
  imuData.has_orientation = false;
  
  return imuData;
}