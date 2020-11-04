#include "WindSensor.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

WindSensor::WindSensor(uint8_t ADC_pin)
{

    // int voltDivRatio, float vMin, float vMax, float SpeedMin, float SpeedMax, bool pwmMode, int Freq
  // A2, 2, 0.4, 2, 0.2, 32.4, true, 3

  int voltDivRatio = 1;
  float vMin = 0.4; 
  float vMax = 2;
  float SpeedMin = 0.2;
  float SpeedMax = 32.4;
  bool pwmMode = true;

  voltMin = vMin / voltDivRatio;
  voltMax = vMax / voltDivRatio;
  analogPin = ADC_pin;
  analogReadResolution(12);
  // analogSetPinAttenuation(pin, attenuation)
  // analogSetSamples(128);
  analogSetPinAttenuation(analogPin, ADC_6db); // Seem  like default is ADC_0db! ADC_11db. Check https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
  speedMin = SpeedMin;
  speedMax = SpeedMax;
  PWM_MODE = pwmMode;
  // analogPin = analogPin;
  Serial.println("Created a Wind sensor");
}

void WindSensor::setup(ProtobufBridge bridge) {
  protobridge = bridge;
}

float WindSensor::mapFloat(float value, float in_min, float in_max, float out_min, float out_max)
{
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Wind WindSensor::prepareData(int64_t time)
{
  Wind windData = Wind_init_zero;
  windData.time = time;

  long sum = 0;
  for (size_t i = 0; i < 10; i++)
  {
    sum += analogRead(analogPin); // perhaps this could be done internally https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html#_CPPv425adc2_config_channel_atten14adc2_channel_t11adc_atten_t
  }
  

  uint16_t sensorRaw = sum/10;

  // map the raw sensor data to the voltage between 0.0 to 3.0
  float sensorVoltage = mapFloat((float) sensorRaw, 475.0F, 3360.0F, 0.3F, 1.6F); // https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
    // float sensorValue = mapFloat((float) sensorRaw, 0F, 1024.0F, 0.195F, 1.0F);

  
  /* 
     * convert the voltage to wind speed 
     * calibration refrence 
     * https://thepihut.com/products/adafruit-anemometer-wind-speed-sensor-w-analog-voltage-output
  */


  windData.speed = mapFloat(sensorVoltage, voltMin, voltMax, speedMin, speedMax);
  windData.direction = -1;

  // Serial.print("Wind : ");
  // Serial.println(windData.speed);
  // Serial.print("Analog read : ");
  // Serial.println(sensorRaw);
  // Serial.print("Raw Volt : ");
  // Serial.println(sensorVoltage);
    
  return windData;
}

void WindSensor::loop() {
  // if( uploadTimer.doRun() ) { // can be used for debugging.
  //   prepareData(0);
  // }
}

void WindSensor::loopWifiAndTime(int64_t time){
  if( uploadTimer.doRun() ) {
    Serial.println("it's time");
    protobridge.sendWind(prepareData(time));
  }
}


// void WindSensor::setupWindDirEncoder()
// {
//   if (PWM_MODE)
//   {
//     // set up the analog pin of the PWM encoder
//     windDirectionPWM_setup(A3);
//     Serial.println("Successfully set up PWM mode!");
//   }
//   else
//   {
//     // connect to the AS5140 sensor
//     if (!windDirEncoder.begin())
//     {
//       Serial.println("Error setting up wind direction encoder");
//     }
//     else
//     {
//       Serial.println("Successfully set up wind direction encoder");
//     }
//   }
// }





  // if (PWM_MODE)
  // {
  //   windData.direction = windDirectionPWM();
  // }
  // else
  // {
  //   windData.direction = windDirection();
  // }



// float WindSensor::windDirection()
// {
//   int rawData = windDirEncoder.read();
//   bool Status = windDirEncoder.valid();
//   float direction = 0.0F;

//   if (Status == true)
//   {
//     direction = mapFloat(rawData, 0, 1024, 0, 360);
//   }
//   else
//   {
//     direction = (float)2000;
//   }

//   return direction;
// }

// void WindSensor::windDirectionPWM_setup(uint8_t ADC_pin)
// {
//   analogPWMPin = ADC_pin;
// }

// float WindSensor::windDirectionPWM()
// {
//   float rawDir = analogRead(analogPWMPin);
//   float PWM_2_ANG = mapFloat(rawDir, 0.0F, 3980.0F, 0.0F, 360.0F);
//   /*
//   Note : the mechanical magnet installation is intended to have that offset. Ask hossein
//   */
//   //    Serial.println(PWM_2_ANG);
//   return PWM_2_ANG;
// }