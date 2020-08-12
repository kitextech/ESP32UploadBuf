#include "PowerSensor.h"

// multi-inputs constructor
PowerSensor::PowerSensor(uint8_t numSamples_, uint8_t voltagePin_, uint8_t currentPin_,
                         float b1_, float m1_, float b2_, float m2_, float bV_, float mV_, float bC_, float mC_,
                         uint16_t uploadFrequency_)
{
  Serial.println("Created a power sensor");
  numSamples = numSamples_;
  voltagePin = voltagePin_;
  currentPin = currentPin_;
  b1 = b1_;
  m1 = m1_;
  b2 = b2_;
  m2 = m2_;
  bV = bV_;
  mV = mV_;
  bC = bC_;
  mC = mC_;
  uploadFrequency = uploadFrequency_;
  t0 = millis();
}
PowerSensor::PowerSensor(uint8_t nSamples, uint8_t vPin, uint8_t cPin,
                         float b1_, float m1_, float b2_, float m2_, float bV_, float mV_, float bC_, float mC_,
                         uint16_t uploadFrequency_, float minVolt, float maxVolt, uint8_t dumpPin1, uint8_t dumpPin2, uint8_t dumpPin3, uint8_t dumpPin4)
{
  Serial.println("Created a power sensor");
  numSamples = nSamples;
  voltagePin = vPin;
  currentPin = cPin;
  b1 = b1_;
  m1 = m1_;
  b2 = b2_;
  m2 = m2_;
  bV = bV_;
  mV = mV_;
  bC = bC_;
  mC = mC_;
  uploadFrequency = uploadFrequency_;
  BatMinVolt = minVolt;
  BatMaxVolt = maxVolt;
  MANUAL_OVERRIDE_SWITCH = dumpPin1;
  GREEN_LED = dumpPin2;
  RED_LED = dumpPin3;
  DigitalPin4 = dumpPin4;

  t0 = millis();
}; // What's this semicolon doing here?

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

  // Serial.print(voltage);
  // Serial.println(" V");
  // Serial.print(current);
  // Serial.println(" A");
  // Serial.print(voltage * current);
  // Serial.println(" W");
  switch (status_indicator)
  {
  case (AUTOMATIC_DUMP_OFF):
    Serial.println("AUTOMATIC POWER DUMPING IS OFF!");
    break;
  case (AUTOMATIC_DUMP_ON):
    Serial.println("AUTOMATIC POWER DUMPING IS ON!");
    break;
  case (MANUAL_DUMP_OFF):
    Serial.println("MANUAL KEY IS ENGAGED!");
    break;
  case (Default):
    Serial.println("BOOTING UP!");
    break;
  };

  sumV = 0;
  sumC = 0;

  data.voltage = voltage;
  data.current = current;

  return data;
}

void PowerSensor::PowerDumpSetup()
{
  pinMode(MANUAL_OVERRIDE_SWITCH, INPUT_PULLDOWN);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(DigitalPin4, OUTPUT);
  status_indicator = Default;
  BootUpTimeStart = millis();
  Serial.println("Successfully set up the output pins for Power Dump system");
  Serial.println(MANUAL_OVERRIDE_SWITCH);
  Serial.println(GREEN_LED);
  Serial.println(RED_LED);
  Serial.println(DigitalPin4);
  Serial.println("Power system set in bootup Mode!");
}

void PowerSensor::PowerControl()
{
  if (millis() - BootUpTimeStart > BootUpTime)
  {
    if (digitalRead(MANUAL_OVERRIDE_SWITCH))
    {
      digitalWrite(DigitalPin4, LOW);
      status_indicator = MANUAL_DUMP_OFF;
    }
    else
    {
      if (voltage > BatMaxVolt)
      {
        // opening the gate
        digitalWrite(DigitalPin4, HIGH);
        status_indicator = AUTOMATIC_DUMP_ON;
        history_indicator = AUTOMATIC_DUMP_ON;
      }
      else if (voltage < BatMinVolt)
      {
        digitalWrite(DigitalPin4, LOW);
        status_indicator = AUTOMATIC_DUMP_OFF;
        history_indicator = AUTOMATIC_DUMP_OFF;
      }
      else
      {
        if (history_indicator == AUTOMATIC_DUMP_ON)
        {
          digitalWrite(DigitalPin4, HIGH);
          status_indicator = AUTOMATIC_DUMP_ON;
        }
        if (history_indicator == AUTOMATIC_DUMP_OFF)
        {
          digitalWrite(DigitalPin4, LOW);
          status_indicator = AUTOMATIC_DUMP_OFF;
        }
      }
    }
  }
  else
  {
    digitalWrite(DigitalPin4, LOW);
    status_indicator = Default;
  }
}

void PowerSensor::Indicator()
{
  switch (status_indicator)
  {

  case (AUTOMATIC_DUMP_ON):
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    BLINKING_FLAG = true;
    break;

  case (AUTOMATIC_DUMP_OFF):
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    BLINKING_FLAG = true;
    break;

  case (Default):
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, HIGH);
    BLINKING_FLAG = true;
    break;

  case (MANUAL_DUMP_OFF):
    if (BLINKING_FLAG)
    {
      initialTime = millis();
      BLINKING_FLAG = false;
    }
    initialTime = Blink(initialTime);
    break;
  }
}

int PowerSensor::Blink(int starttime)
{
  if ((millis() - starttime) > 1000 / FreqLED)
  {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, HIGH);
  }
  else
  {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW);
  }
  if ((millis() - starttime) > 2000 / FreqLED)
  {
    starttime = millis();
  }
  return (starttime);
}