// #include <Arduino.h>
// #include "msg.pb.h"
// #include "schema.pb.h"
// #include "pb_common.h"
// #include "pb.h"
// #include "pb_encode.h"


// uint8_t buffer[128];
// size_t imuMessageLength;
// uint8_t bufferWrapper[512];

// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(115200);
// }

// bool write_imuBuffer(pb_ostream_t *stream, const pb_field_iter_t *field, void * const *arg)
// {
//     //char *str = "Hello world!";
    
//     if (!pb_encode_tag_for_field(stream, field))
//         return false;
    
//     return pb_encode_string(stream, (uint8_t*) &buffer, imuMessageLength);
// }

// void loop() {
//   // put your main code here, to run repeatedly:

//   delay(1000);

//   pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

//   Quaternion orientation = Quaternion_init_zero; // or {1,0,0,1}
//   orientation.x = 1;
//   orientation.y = 0;
//   orientation.z = 0;
//   orientation.w = 1;

//   Vector3 acc = {0, 0.2, 9.82};
//   Vector3 gyro = {0,0,1};

//   Imu imu = Imu_init_zero;
//   imu.has_acc = true;
//   imu.acc = acc;
//   imu.has_gyro = true;
//   imu.gyro = gyro;
//   imu.has_orientation = true;
//   imu.orientation = orientation;
//   imu.time = int64_t(millis() + 1e12 );

//   // write IMU data
//   bool status = pb_encode(&stream, Imu_fields, &imu);
 
//   if (!status)
//   {
//       Serial.println("Failed to encode imu");
//       return;
//   }
 
//   Serial.print("Message Length imu: ");
//   Serial.println(stream.bytes_written);
//   imuMessageLength = stream.bytes_written;
  

//   stream = pb_ostream_from_buffer(bufferWrapper, sizeof(bufferWrapper));

//   // wrapper
//   Wrapper wrap = Wrapper_init_zero;
//   wrap.type = Wrapper_DataType_IMU;
//   wrap.data.funcs.encode = &write_imuBuffer;

//   status = pb_encode(&stream, Wrapper_fields, &wrap);
 
//   if (!status)
//   {
//       Serial.println("Failed to encode wrapper");
//       return;
//   }
 
//   Serial.print("Message Length wrapper: ");
//   Serial.println(stream.bytes_written);
 
//   Serial.print("Message wrapper: ");
 
//   for(int i = 0; i<stream.bytes_written; i++){
//     Serial.printf("%02X",bufferWrapper[i]);
//   }

//   Serial.println();
// }