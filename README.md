# ESP32UploadBuf
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
## Description
At KiteX, we are developing a new type of ultra lightweight wind turbine. This repository contains the firmware for the microcontrollers used for data collection and control of the turbine. The idea is to have an efficient IoT setup that allows synchronized time-series data collection from multiple sources. The data is stored in an InfluxDB 2.0 database and accessed via Grafana.

This drone footage from [one of our latest tests](https://www.linkedin.com/feed/update/urn:li:activity:6676809609837121536) shows the turbine in action. At this stage, we are actively controlling the RPM from a laptop.

![](https://github.com/kitextech/ESP32UploadBuf/blob/readme/kitex-twt.gif?raw=true)

The image below shows an overview of the system. [You can find the server and laptop side of the system here.](https://github.com/kitextech/openTWTLogging)

![](https://github.com/kitextech/ESP32UploadBuf/blob/readme/esp32uploadbuf.png)

Functionalities of the present firmware include:
* Collect data from a variety of sensors (load cells, IMU etc.)
* Sync time-series data using a common time server
* Encode the data via [Protocol Buffers](https://developers.google.com/protocol-buffers)
* Upload the data to an InfluxDB 2.0 server using a UDP socket
* Receive RPM setpoint values over TCP and send them to an ESC

This project is based on [this IoT project by Vladimir Vivien](https://medium.com/grpc/efficient-iot-with-the-esp8266-protocol-buffers-grafana-go-and-kubernetes-a2ae214dbd29), and we recommend you read the writeup as it explains many of the core principles.

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
