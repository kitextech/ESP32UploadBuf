#include "VescControl.h"

#define DUMP_PIN A12

VescControl::VescControl(boolean doDump, boolean rc_overwrite) :
doDump(doDump),
rc_overwrite(rc_overwrite)
{
  if (doDump) {
    pinMode(A12, OUTPUT); // Outpit pin A12
    digitalWrite(DUMP_PIN, HIGH); // Output is inverted using a transitor to pull Mosfet High.
    Serial.println("Set up the output pins for Power Dump system");
  }
}

void VescControl::setup(ProtobufBridge proto) //Fixme pass by value. Lucky that it work
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
  setpointData.RPM = rpmsetpoint;//rpm_sp;
  setpointData.current = currentSetpoint;
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
    controlVESC();
  }

  if (doDump && controlDump.doRun()){
    controlDumping(vesc.data.inpVoltage);
  }
}


void VescControl::controlVESC() {
  vesc.getVescValues();

  if (rc_overwrite) {
    return;
  }

  switch (control.command)
  {
  case TurbineControl_Command_Stop:
    vesc.setDuty(0);
    break;
  case TurbineControl_Command_Auto:
    /* code */
    speedController.doControlV1(vesc, currentSetpoint, control.value);
    rpmsetpoint = (float) vesc.data.rpm;
    break;
  case TurbineControl_Command_Speed: 
    {
      // change the rpm by 200 eRPM every time the function is run.. // set the RPM at maximum 200 eRPM from the current eRPM
      float rpmTarget = control.value*RPM_TO_ERPM;
;

      // long rpmsetpoint = (long) control.value;
      float delta = rpmTarget - rpmsetpoint;
      delta = min(delta, 200.0f);
      delta = max(-200.0f, delta);
      rpmsetpoint = rpmsetpoint + delta;
      
      // if (rpmsetpoint > 1500 && target <= 1500) {
      //   target = 1500;
      // }
      // rpmsetpoint = control.value;

      vesc.setRPM( rpmsetpoint );
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

void VescControl::controlDumping(float voltage) {
  if (voltage > 3.8*12) {
    digitalWrite(DUMP_PIN, LOW); // Output is inverted using a transitor to pull Mosfet High
    Serial.println("Dumping On");
  } 

  if (voltage < 3.5*12) {
    digitalWrite(DUMP_PIN, HIGH); // Output is inverted using a transitor to pull Mosfet High
      Serial.println("Dumping Off");
  } 
}

void VescControl::loopWifiAndTime(int64_t time){
  
  if (uploadData.doRun()) {
    //send vesc data to influx using protobridge
    protobridge.sendVesc(prepareVescData(time));
    protobridge.sendSetpoint(prepareSetpointData(time));
  }


}