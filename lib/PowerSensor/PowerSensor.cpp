#include "PowerSensor.h"

// multi-inputs constructor
PowerSensor::PowerSensor(uint8_t numSamples, uint8_t voltagePin, uint8_t currentPin,
                         float b1, float m1, float b2, float m2, float bV, float mV, float bC, float mC,
                         uint16_t uploadFrequency)
{
  Serial.println("Created a power sensor");
  numSamples = numSamples;
  voltagePin = voltagePin;
  currentPin = currentPin;
  b1 = b1;
  m1 = m1;
  b2 = b2;
  m2 = m2;
  bV = bV;
  mV = mV;
  bC = bC;
  mC = mC;
  uploadFrequency = uploadFrequency;
  t0 = millis();
}
PowerSensor::PowerSensor(uint8_t numSamples, uint8_t voltagePin, uint8_t currentPin,
                         float b1, float m1, float b2, float m2, float bV, float mV, float bC, float mC,
                         uint16_t uploadFrequency, float minVolt, float maxVolt, uint8_t dumpPin1, uint8_t dumpPin2, uint8_t dumpPin3, uint8_t dumpPin4)
{
  Serial.println("Created a power sensor");
  numSamples = numSamples;
  voltagePin = voltagePin;
  currentPin = currentPin;
  b1 = b1;
  m1 = m1;
  b2 = b2;
  m2 = m2;
  bV = bV;
  mV = mV;
  bC = bC;
  mC = mC;
  uploadFrequency = uploadFrequency;
  BatMinVolt = minVolt;
  BatMaxVolt = maxVolt;
  DigitalPin1 = dumpPin1;
  DigitalPin2 = dumpPin2;
  DigitalPin3 = dumpPin3;
  DigitalPin4 = dumpPin4;

  t0 = millis();
};  // What's this semicolon doing here?

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

void PowerSensor::PowerDumpSetup()
{
  pinMode(DigitalPin1, OUTPUT);
  pinMode(DigitalPin2, OUTPUT);
  pinMode(DigitalPin3, OUTPUT);
  pinMode(DigitalPin4, OUTPUT);
  Serial.println("Successfully set up the output pins for Power Dump system");
}
void PowerSensor::PowerControl()
{
  if (voltage > BatMaxVolt)
  {
    digitalWrite(DigitalPin1, HIGH);
    digitalWrite(DigitalPin2, HIGH);
    digitalWrite(DigitalPin3, HIGH);
    digitalWrite(DigitalPin4, HIGH);
  }
  if (voltage < BatMinVolt)
  {
    digitalWrite(DigitalPin1, LOW);
    digitalWrite(DigitalPin2, LOW);
    digitalWrite(DigitalPin3, LOW);
    digitalWrite(DigitalPin4, LOW);
  }
}