/*
Based on Adafruit Learning System guide on Thermistors by Limor Fried, Adafruit Industries
Link:  https://learn.adafruit.com/thermistor/overview

Modified by Bertalan Kovács (bertalan@kitex.tech) and Mathias Neuenschwander (mathias@kitex.tech)
*/

#include "TemperatureSensor.h"

TemperatureSensor::TemperatureSensor(uint16_t uploadFreq, uint8_t pinTherm, uint16_t nomTherm, int8_t nomTemp, \
  uint16_t beta, uint16_t rSeries) : pinTherm(pinTherm), nomTherm(nomTherm), nomTemp(nomTemp), beta(beta), rSeries(rSeries)
{
  uploadFrequency = uploadFreq;
  
  t0 = millis();
  Serial.println("Successfully created a Temperature sensor handling object");
}

// void TemperatureSensor::setup()
// {
// }


Temperature TemperatureSensor::prepareData(int64_t time)
{
  Temperature temperatureData = Temperature_init_zero;
  temperatureData.time = time;

  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i = 0; i < TEMP_SENS_NUM_SAMPLES; i++)
  {
    adc_samples[i] = analogRead(pinTherm);
  }

  // average all the samples out
  average = 0;
  for (i = 0; i < TEMP_SENS_NUM_SAMPLES; i++)
  {
    average += adc_samples[i];
  }
  average /= TEMP_SENS_NUM_SAMPLES;

  // convert the value to resistance
  average = 1023 / average - 1;
  average = rSeries / average;

  float steinhart;
  steinhart = average / nomTherm;          // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= beta;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (nomTemp + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;                              // convert to C

  Serial.print("Temperature: ");
  Serial.print(steinhart);
  Serial.println(" °C");

  temperatureData.temperature = steinhart;

  return temperatureData;
}