#include "BladePitchControl.h"

// constructor
BladePitchControl::BladePitchControl(int _pin1, int _pin2, int _pin3)
{
  Serial.println("Created a pitch controller");
  pin1 = _pin1;
  pin2 = _pin2;
  pin3 = _pin3;
}

BladeControl BladePitchControl::prepareData(int64_t time)
{

  BladeControl data = BladeControl_init_zero;
  data.time = time;

  data.pitch1 = pitch1;
  data.pitch2 = pitch2;
  data.pitch3 = pitch3;
  data.collectivePitch = collectivePitch;
  return data;
}

void BladePitchControl::setup(ProtobufBridge bridge)
{ 
  protobridge = bridge;

  int minUs = 800;
  int maxUs = 2200;
  Serial.println("Blade Setup");
  ESP32PWM pwm; // don't think it does anythin
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	servo1.setPeriodHertz(50);      // Standard 50hz servo
	servo2.setPeriodHertz(50);      // Standard 50hz servo
	servo3.setPeriodHertz(50);      // Standard 50hz servo
  servo1.attach(pin1, minUs, maxUs);
	servo2.attach(pin2, minUs, maxUs);
	servo3.attach(pin3, minUs, maxUs);
}

void BladePitchControl::loopWifiAndTime(int64_t time) {
    if( uploadTimer.doRun() ) {
    // prepareData(time);
    protobridge.sendBladeControl(prepareData(time));
  }
}


// Limites the input. in Blade for the servos
float BladePitchControl::limit(float value, float min, float max) {
  return _min(_max(value, min), max);
}


void BladePitchControl::loop(uint8_t UDPInBuffer[], int n)
{
  // Serial.print("Server to client: ");
  // int i;
  // for (i = 0; i < n; i++)
  // {
  //     if (i > 0) printf(":");
  //     printf("%02X", UDPInBuffer[i]);
  // }
  // printf("\n");

  // Serial.printf("time: %lld\n", newLocalTime());

  // // decode the buffer
  // Wrapper message = Wrapper_init_zero;
  // pb_istream_t stream = pb_istream_from_buffer(buffer, 128);
  // bool status = pb_decode(&stream, Wrapper_fields, &message); 

  // if (!status)
  // {
  //   Serial.printf("Decoding of wrapper failed: %s\n", PB_GET_ERROR(&stream));
  // }

  // decode the Blade
  // if (message.type != Wrapper_DataType_BLADE)
  // {
  //   Serial.printf("Unexpetect Type: %n\n", message.type);
  // }

  BladeControl bcMessage = BladeControl_init_zero;
  pb_istream_t cbStream = pb_istream_from_buffer(UDPInBuffer, n);
  bool status = pb_decode(&cbStream, BladeControl_fields, &bcMessage);

  if (!status)
  {
    // Serial.printf("Decoding failed: %s\n", PB_GET_ERROR(&cbStream));
  } 
  else {
    // Serial.printf("Excelent: pitch1: %f, pitch2: %f, pitch3: %f, collective: %f\n", bcMessage.pitch1,  bcMessage.pitch2,  bcMessage.pitch3,  bcMessage.collectivePitch);
    
    pitch1 = bcMessage.pitch1;
    pitch2 = bcMessage.pitch2;
    pitch3 = bcMessage.pitch3;
    collectivePitch = bcMessage.collectivePitch;

    float servo1Out = (bcMessage.pitch1 + bcMessage.collectivePitch)*500 + 1500;
    float servo2Out = (bcMessage.pitch2 + bcMessage.collectivePitch)*500 + 1500;
    float servo3Out = (bcMessage.pitch3 + bcMessage.collectivePitch)*500 + 1500;

    servo1.writeMicroseconds( limit(servo1Out, 900,2100) );
    servo2.writeMicroseconds( limit(servo2Out, 900,2100) );
    servo3.writeMicroseconds( limit(servo3Out, 900,2100) );

  }

}