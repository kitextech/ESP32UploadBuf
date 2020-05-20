#include "PowerSensor.h"

PowerSensor::PowerSensor(uint8_t numSamples, uint8_t voltagePin, uint8_t currentPin, \
  float b1, float m1, float b2, float m2, float bV, float mV, float bC, float mC, \
  uint16_t uploadFrequency)
  // : numSamples(numSamples), voltagePin(voltagePin), currentPin(currentPin), b1(b1), m1(m1), b2(b2), m2(m2), bV(bV), mV(mV), bC(bC), mC(mC), uploadFrequency(uploadFrequency)
{
  Serial.println("Created a power sensor");
  // numSamples = numSamples;
  // voltagePin = voltagePin;
  // currentPin = currentPin;
  // b1 = b1;
  // m1 = m1;
  // b2 = b2;
  // m2 = m2;
  // bV = bV;
  // mV = mV;
  // bC = bC;
  // mC = mC;
  // uploadFrequency = uploadFrequency;
  t0 = millis();
}

Power PowerSensor::prepareData(int64_t time)
{
  Power data = Power_init_zero;
  data.time = time;

  // take a number of analog samples and add them up
  for (uint8_t i = 0; i < numSamples; i++)
  {
    sumV += analogRead(voltagePin);
    sumC += analogRead(currentPin);
  }

  // calculate the voltage
  // 12-bit ADC sensors, 3.3V sensor signal
  adcVoltage = ((float)sumV / (float)numSamples) * 3.3 / 4095.0;
  adcCurrent = ((float)sumC / (float)numSamples * 3.3) / 4095.0;

  // correction based on oscilloscope measurement and linear regression (only ADC of the ESP32)
  if (adcVoltage > 0 && adcVoltage < 2.6)
    adcVoltage = adcVoltage * m1 + b1;
  else if (adcVoltage >= 2.6)
    adcVoltage = adcVoltage * m2 + b2;
  
  if (adcCurrent > 0 && adcCurrent < 2.6)
    adcCurrent = adcCurrent * m1 + b1;
  else if (adcCurrent >= 2.6)
    adcCurrent = adcCurrent * m2 + b2;

  // multiplyer factors based on further measurements (datasheet was only correct for the voltage)
  voltage = adcVoltage * mV + bV;
  current = adcCurrent > 0 ? adcCurrent * mC + bC : 0;

  Serial.print(voltage);
  Serial.println(" V");
  Serial.print(current);
  Serial.println(" A");
  Serial.print(voltage * current);
  Serial.println(" W");

  sumV = 0;
  sumC = 0;

  data.voltage = voltage;
  data.current = current;

  return data;
}