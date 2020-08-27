#include "VescControl.h"

VescControl::VescControl(uint16_t uploadFreq) :
checkFirebase(20000),
uploadFrequency(uploadFreq)
{
  t0 = millis();
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
    rpm_sp_udp = message.RPM;
  }
}

void VescControl::updateRpmSetpoint(float RPM) {
    if (RPM != rpmSetpoint)
    {
      Serial.printf("RPM is set to %d\n", (int) RPM);  
      rpmRampStart = vesc.data.rpm;
      rpmDiff = RPM - rpmRampStart;
      t0_ramp = millis();
      rampingTime = abs((int)(rpmDiff * 1.0/rampAcc));
      rpmSetpoint = RPM;
    }
}

void VescControl::setRpm()
{
    // shoi
    if (firebaseControlDoc["active"]) {
      updateRpmSetpoint(firebaseControlDoc["speed"].as<float>()*turbineGearRatio);
    } else {
      updateRpmSetpoint(rpm_sp_udp);
    }

    vesc.getVescValues(); // could change to an algorithm that isn't sensitive to delays
    if (t0_ramp + rampingTime > millis())
      rpm_sp = (float(millis()) - float(t0_ramp)) / rampingTime * rpmDiff + ((float)rpmSetpoint - rpmDiff);
    else
      rpm_sp = rpmSetpoint;

    vesc.setRPM(rpm_sp);
    
    // temp
    //runAsyncClient();
}



void VescControl::runFirebaseCheck() {

  unsigned long start = millis();
  Serial.print("[HTTP] begin...\n");
  // configure traged server and url
  //http.begin("https://www.howsmyssl.com/a/check", ca); //HTTPS
  http.useHTTP10(true); // since we deserialize directly. // https://arduinojson.org/v6/how-to/use-arduinojson-with-esp8266httpclient/
  
  //http.begin("https://opentwt-6eadd.firebaseio.com/SpeedControl.json"); //HTTP
  http.begin("https://opentwt-6eadd.firebaseio.com/SpeedControl.json"); //HTTP
  //http.setTimeout() default timeout is 5000 ms.
  
  // Serial.print("[HTTP] GET...\n");
  // start connection and send HTTP header
  int httpCode = http.GET();

  // httpCode will be negative on error
  if(httpCode > 0) {

      // file found at server
      if(httpCode == HTTP_CODE_OK) {
          // String payload = http.getString(); // can not get string and get stream!
          // Serial.println(payload);
          deserializeJson(firebaseControlDoc, http.getStream());
          //serializeJsonPretty(doc, Serial);
      } else {
        // HTTP header has been send and Server response header has been handled. Print non OK Code
        Serial.printf("[HTTP] GET... code: %d\n", httpCode);

      }
  } else {
      Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
  Serial.println(millis()-start);
}


void VescControl::runFirebaseCheckAsync() {

}