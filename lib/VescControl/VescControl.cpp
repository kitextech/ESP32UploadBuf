#include "VescControl.h"

VescControl::VescControl(uint16_t uploadFreq)
{
  uploadFrequency = uploadFreq;
  t0 = millis();
  modeArrayLength = 5;
}

void VescControl::setup()
{
  SerialVesc.begin(115200, SERIAL_8N1, 16, 17);
  vesc.setSerialPort(&SerialVesc);
  // Serial1.begin(115200);  // rx/tx pins of ESP32 (for the vesc)
  // vesc.setSerialPort(&Serial1);
}

Vesc VescControl::prepareVescData(int64_t time)
{
  Vesc vescData = Vesc_init_zero;
  vescData.time = time;
  vescData.avgMotorCurrent = vesc.data.avgMotorCurrent;
  vescData.avgInputCurrent = vesc.data.avgInputCurrent;
  vescData.dutyCycleNow = vesc.data.dutyCycleNow;
  vescData.rpm = vesc.data.rpm;
  vescData.inpVoltage = vesc.data.inpVoltage;
  vescData.ampHours = vesc.data.ampHours;
  vescData.ampHoursCharged = vesc.data.ampHoursCharged;
  vescData.tachometer = vesc.data.tachometer;
  vescData.tachometerAbs = vesc.data.tachometerAbs;
  return vescData;
}

Setpoint VescControl::prepareSetpointData(int64_t time)
{
  Setpoint setpointData Setpoint_init_zero;
  setpointData.time = time;
  setpointData.RPM = rpm_sp;
  setpointData.current = 0;
  return setpointData;
}

int VescControl::mode(int a[], int n) {
   int maxValue = 0, maxCount = 0, i, j;
   for (i = 0; i < n; ++i) {
      int count = 0;
      for (j = 0; j < n; ++j) {
         if (a[j] == a[i])
         ++count;
      }
      if (count > maxCount) {
         maxCount = count;
         maxValue = a[i];
      }
   }
   return maxValue;
}

void VescControl::updateArray(int newElement, int n)
{
  for (int i = n-1; i > 0; --i) {
    rpmSetpointArray[i] = rpmSetpointArray[i-1] ;
  } 
  rpmSetpointArray[0] = newElement;
}

void VescControl::updateRpmSetpoint(WiFiClient client)
{
  if (client)
  {
    while (client.connected())
    {
      if (client.available() > 0)
      {
        client.read(bufferTCP, 1);
        // String str = String("Message length (bytes): ") + (bufferTCP[0]);
        // Serial.println(str);

        int msg_length = bufferTCP[0];

        client.read(bufferTCP, bufferTCP[0]);
        Speed message = Speed_init_zero;
        pb_istream_t stream = pb_istream_from_buffer(bufferTCP, msg_length);
        bool status = pb_decode(&stream, Speed_fields, &message);

        if (!status)
        {
          Serial.printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));
        }
        else
        {
          updateArray((int)(message.RPM), MODE_ARRAY_LENGTH);
          if (message.RPM == mode(rpmSetpointArray, MODE_ARRAY_LENGTH) && message.RPM != rpmSetpoint)
          {
            Serial.printf("RPM is set to %d\n", (int)message.RPM);  
            rpmDiff = message.RPM - (float)rpmSetpoint;
            t0_ramp = millis();
            rampingTime = abs((int)(rpmDiff * 1.0/rampAcc));
            rpmSetpoint = (float)mode(rpmSetpointArray, MODE_ARRAY_LENGTH);
          }
        }
      }
      return;
    }
  }
}

void VescControl::setRpm()
{
    vesc.getVescValues();
    if (t0_ramp + rampingTime > millis())
      rpm_sp = (float(millis()) - float(t0_ramp)) / rampingTime * rpmDiff + ((float)rpmSetpoint - rpmDiff);
    else
      rpm_sp = rpmSetpoint;
    Serial.printf("RPM: %f\n", rpm_sp);  
    vesc.setRPM(rpm_sp);
}