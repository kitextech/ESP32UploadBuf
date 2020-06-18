# ESP32UploadBuf

## Description
The overall purpose is to have an efficient IoT setup that allows synchronized time-series data collection from multiple sources using ESP's. The data is then stored in an InfluxDB 2.0 database and accessed via Grafana.

This repository contains a shared codebase for several microcontrollers that share common functionality (data logging to the same server) but serve different functions. Specific ESP examples included:
* Collect wind speed and direction data
* Collect data from load cells
* Collect temperature data
* Collect IMU data
* Control an [ESC](https://en.wikipedia.org/wiki/Electronic_speed_control) and log data from it
* *etc*.

Shared functionality:
* Connect to the same InfluxDB server on the same WiFi network
* Encode the collected data using [Protocol Buffers](https://developers.google.com/protocol-buffers)
* Send the data using a UDP socket
* Sync time-series data using a common time server

![](https://github.com/kitextech/ESP32UploadBuf/blob/readme/esp32uploadbuf.png)

[The server-side repository is available here.
](https://github.com/kitextech/openTWTLogging)

This project is based on [this IoT project by Vladimir Vivien](https://medium.com/grpc/efficient-iot-with-the-esp8266-protocol-buffers-grafana-go-and-kubernetes-a2ae214dbd29), and we recommend you read the writeup as it explains many of the core principles. The main differences between this project and Vivien's are:

1. This project supports using the same firmware for multiple ESP's that need to communicate with the same server
2. Instead of TCP, UDP is used for uploading data
3. We use esp32's instead of an esp8266
4. TCP is implemented for server --> client communication for e.g controlling an actuator from the server

## Prerequisites
The project uses the Espressif 32 development platform and the Arduino framework. In order to build it you will need to [use Platform IO, which is available as an extension for VS Code and CLion](https://docs.platformio.org/en/latest/integration/ide/pioide.html). If you don't wish to install the IDE/extension, you can use the command-line interface ([click here to see an example](https://github.com/platformio/platform-espressif32/blob/master/examples/arduino-blink/README.rst)).

## Installation
### VS Code
1. Install the Platform IO extension
2. Download the project files and open the project
3. Build the project using the Platform IO tool bar button or keyboard shortcut (`Ctrl+Alt+B` on Windows). During the first build, the libraries defined in `lib_deps` in the `platformio.ini` file will be installed to the `.pio` folder (except the ones shipped with the project in the `lib` folder) along with platform specific build files.

## Usage
### Setup
All necessary setup for the current sensors is done in the `init.h` file. Use the `DEFINE` statements to include the desired modules; the rest will not be compiled and uploaded to the microcontroller. The `init.h` file is also used to setup the desired network and ports.

The development platform (Espressif 32) and framework (Arduino) are defined in `platformio.ini` along with upload and monitor ports and library dependencies (incl. the contents of the `lib` folder).

### Modifications
The data is formatted using protocol buffers. To that end, the project specific `ProtobufBridge` library is used. The library contains a send function for each of the protobuf message types defined in the `pb.c` and `pb.h` files. If you wish to add a message template  (e.g. for another sensor type), you will need to generate new `.pb.h` and `.pb.c` files. These can be generated from a `.proto` file using [a modified version](https://github.com/kitextech/ESPNanopb) of the [nanopb library](https://github.com/nanopb/nanopb)  which contains the messages used in this project.

The sensor modules are located in the `lib` folder, and each of them includes a function `prepareData()` which collects the data and creates the protobuf message from it. The protobuff messages are sent to the server via UDP in `main.cpp` at frequencies defined upon creation of the sensor objects in the `init.h` file.

## Directory structure
The basic directory structure is defined by Platform IO:

```bash
├───.pio
│   ├───build
│   │   └───featheresp32
│   └───libdeps
│       └───featheresp32
├───include
├───lib
│   ├───AS5040
│   ├───ForceSensor
│   ├───HallSensor
│   ├───ImuSensor
│   ├───MedianFilter
│   ├───PowerSensor
│   ├───ProtobufBridge
│   ├───TemperatureSensor
│   └───WindSensor
├───src
├───test
└───tools
```

## License
Maybe this one? [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
