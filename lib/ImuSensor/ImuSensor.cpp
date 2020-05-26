#include "ImuSensor.h"

ImuSensor::ImuSensor(int uploadFreq)
{
  uploadFrequency = uploadFreq;
  t0 = millis();
  Serial.println("Created an IMU sensor handling object");
}

void ImuSensor::setup()
{ // BNO-055 SETUP
  Serial.println("Orientation Sensor Raw Data Test");
  Serial.println("");
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  delay(1000);
  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");
  bno.setExtCrystalUse(true);
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

Imu ImuSensor::prepareData(int64_t time)
{
  Imu imuData = Imu_init_zero;
  imuData.time = time;

  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Quaternion quat = bno.getQuat();

  imuData.has_acc = true;
  imuData.acc.x = acc.x();
  imuData.acc.y = acc.y();
  imuData.acc.z = acc.z();

  imuData.has_gyro = true;
  imuData.gyro.x = gyro[0];
  imuData.gyro.y = gyro.y();
  imuData.gyro.z = gyro.z();

  imuData.has_orientation = true;
  imuData.orientation.x = quat.x();
  imuData.orientation.y = quat.y();
  imuData.orientation.z = quat.z();
  imuData.orientation.w = quat.w();
  
  return imuData;
}