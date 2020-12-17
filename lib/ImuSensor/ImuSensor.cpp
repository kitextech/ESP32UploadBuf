#include "ImuSensor.h"
#define REDLED 13

ImuSensor::ImuSensor()
{
  Serial.println("Created an IMU sensor handling object");
}

void ImuSensor::setup(ProtobufBridge bridge)
{ // BNO-055 SETUP
  protobridge = bridge;

  Serial.println("Orientation Sensor Raw Data Test");
  Serial.println("");
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_AMG))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
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

  // power led - hacky
  pinMode(REDLED, OUTPUT); //RED LED
}

void ImuSensor::loopWifiAndTime(int64_t time){
  if( uploadTimer.doRun() ) {
    // Serial.println("it's time");
    protobridge.sendAccGyro(prepareData(time));
  }
}


AccGyro ImuSensor::prepareData(int64_t time)
{
  AccGyro accGyroData = AccGyro_init_zero;
  accGyroData.time = (uint64_t) time; //220; //
  timeCount +=1; 

  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); // when in nonfsion mode!
  // imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // imu::Quaternion quat = bno.getQuat();

  accGyroData.has_acc = true;
  accGyroData.acc.x = acc.x();
  accGyroData.acc.y = acc.y();
  accGyroData.acc.z = acc.z();

  accGyroData.has_gyro = true;

  accGyroData.gyro.x = gyro.x(); // timeCount%2 ? xx : xxx; //(float) gyro.x();
  accGyroData.gyro.y = gyro.y();
  accGyroData.gyro.z = gyro.z(); //timeCount;

  // if (!(accGyroData.gyro.x < 10 && -10 < accGyroData.gyro.x)) {
  //   Serial.println(accGyroData.gyro.x);
  // } 
  // Serial.print(accGyroData.gyro.x);
  // Serial.print(accGyroData.gyro.y);
  // Serial.println(accGyroData.gyro.z);

  // imuData.has_orientation = true; // when in nonfsion mode!
  // imuData.orientation.x = 0;//quat.x();
  // imuData.orientation.y = 0;//quat.y();
  // imuData.orientation.z = 0;//quat.z();
  // imuData.orientation.w = 0;//quat.w();

  

  // if (statusTimer.doRun()) {
  // //   displaySensorDetails();
  // //   displaySensorStatus();
  // //   displayCalStatus();
  //   float voltage =  ((float) analogRead(A13)) / 4096.0 * 3.3 * 2; // appximate Voltage
  //   Serial.println(voltage); 
  //   if (voltage < (3.5)) {
  //     digitalWrite(REDLED, !digitalRead(REDLED));
  //   }
  // }
  
  return accGyroData;
}

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void ImuSensor::displaySensorDetails()
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void ImuSensor::displaySensorStatus()
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void ImuSensor::displayCalStatus()
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
