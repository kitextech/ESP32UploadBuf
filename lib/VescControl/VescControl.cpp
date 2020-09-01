#include "VescControl.h"

VescControl::VescControl()
{
}

void VescControl::setup(ProtobufBridge proto)
{
  SerialVesc.begin(115200, SERIAL_8N1, 16, 17);
  vesc.setSerialPort(&SerialVesc);
  control = TurbineControl_init_default;
  control.command = TurbineControl_Command_Stop; // initialize in stop mode.
  protobridge = proto;
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

void VescControl::updateTurbineControl(uint8_t UDPInBuffer[], int n)
{

  TurbineControl message = TurbineControl_init_zero;

  pb_istream_t stream = pb_istream_from_buffer(UDPInBuffer, n);
  bool status = pb_decode(&stream, TurbineControl_fields, &message);

  if (!status)
  {
    Serial.printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));
  }
  else
  {
    control = message;
  }
}

void VescControl::loop(){
  // read values from VESC
  if (readVesc.doRun()) {
    vesc.getVescValues();
    
    switch (control.command)
    {
    case TurbineControl_Command_Stop:
      vesc.setDuty(0);
      break;
    case TurbineControl_Command_Auto:
      /* code */
      break;
    case TurbineControl_Command_Speed: 
      {
        // set the RPM at maximum 100 eRPM from the current eRPM
        long rpmsetpoint = (long) control.value;
        int delta = (int) (rpmsetpoint - vesc.data.rpm);
        delta = min(delta, 100);
        delta = max(-100, delta);
        float target = (float) (vesc.data.rpm + delta);
        
        if (rpmsetpoint > 1500 && target <= 1500) {
          target = 1500;
        }
        vesc.setRPM( target );
        break;
      }
    case TurbineControl_Command_Current:
      vesc.setCurrent(control.value);
      break;
    
    case TurbineControl_Command_Pos:
        // not implemented yet!
      break;
    
    default:
      break;
    }
  }
}

void VescControl::loopWifiAndTime(int64_t time){
  
  if (uploadData.doRun()) {
    //send vesc data to influx using protobridge
    protobridge.sendVesc(prepareVescData(time));
  }


}