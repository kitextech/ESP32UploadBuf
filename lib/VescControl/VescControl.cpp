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

void VescControl::updateRpmSetpoint(uint8_t UDPInBuffer[], int n)
{

  Setpoint message = Setpoint_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(UDPInBuffer, n);
  bool status = pb_decode(&stream, Setpoint_fields, &message);

  if (!status)
  {
    Serial.printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));
  }
  else
  {
    if (message.RPM != rpmSetpoint)
    {
      Serial.printf("RPM is set to %d\n", (int)message.RPM);  
      rpmRampStart = vesc.data.rpm;
      rpmDiff = message.RPM - rpmRampStart;
      t0_ramp = millis();
      rampingTime = abs((int)(rpmDiff * 1.0/rampAcc));
      rpmSetpoint = message.RPM;
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

    vesc.setRPM(rpm_sp);
}