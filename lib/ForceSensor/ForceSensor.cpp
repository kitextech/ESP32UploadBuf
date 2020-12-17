#include "ForceSensor.h"

ForceSensor::ForceSensor(int32_t ID, byte dout_, byte pd_sck_, float regC_, float offset_)
{
  id = ID;
  dout = dout_;
  pd_sck = pd_sck_;
  regC = regC_;
  t0 = millis();
  Serial.println("Created a force sensor");
}

void ForceSensor::setup(ProtobufBridge bridge)
{
  protobridge = bridge;
  sensor.begin(dout, pd_sck);
  Serial.printf("dout=%f\n", (float) dout);
}

Force ForceSensor::prepareData(int64_t time)
{
  Force data = Force_init_zero;
  data.time = time;
  if (sensor.is_ready())
  {
    data.force = (float) sensor.read() * regC + offset;
    data.id = id;
    // Serial.printf("id = %d", id);
    // Serial.printf(" has Force = %f\n", data.force);
  }
  return data;
}

void ForceSensor::loopWifiAndTime(int64_t time){
  if( doUpload.doRun() ) {
    // Serial.println("it's time");
    protobridge.sendForce(prepareData(time));
  }
}