![GitHub top language](https://img.shields.io/github/languages/top/effectsmachine/ugv_base_ros) ![GitHub language count](https://img.shields.io/github/languages/count/effectsmachine/ugv_base_ros)
![GitHub code size in bytes](https://img.shields.io/github/languages/code-size/effectsmachine/ugv_base_ros)
![GitHub repo size](https://img.shields.io/github/repo-size/effectsmachine/ugv_base_ros) ![GitHub](https://img.shields.io/github/license/effectsmachine/ugv_base_ros) ![GitHub last commit](https://img.shields.io/github/last-commit/effectsmachine/ugv_base_ros)

# Waveshare UGV Robots

This is a lower computer example for the [Waveshare](https://www.waveshare.com/) UGV robots with ROS Driver: **UGV Rover**, **UGV Beast**, **RaspRover**, **UGV02**\*.

\*The old version of UGV02 is driven by General Driver.

![Image of Rover](./README_footage/UGV-Rover-details-23.jpg)

## Basic Description

The Waveshare UGV robots utilize both an upper computer and a lower computer. This repository contains the program running on the lower computer, which is typically a ESP32 on **ROS Driver for Robots**.  

The program running on the lower computer is either named [ugv_base_ros](https://github.com/effectsmachine/ugv_base_ros.git) or [ugv_base_general](https://github.com/effectsmachine/ugv_base_general.git) depending on the type of robot driver being used.  

The upper computer communicates with the lower computer (the robot's driver based on ESP32) by sending JSON commands via GPIO UART. The host controller, which employs a [Jetson Orin](https://github.com/waveshareteam/ugv_jetson) or a [Raspberry Pi](https://github.com/waveshareteam/ugv_rpi) based on the type of upper computer being used, handles AI vision and strategy planning, while the sub-controller, utilizing an ESP32, manages motion control and sensor data processing. This setup ensures efficient collaboration and enhanced performance.

## Features

- Closed-loop Speed Control with PID
- Web App Based on ESP32
- IMU
- OLED Screen
- LED Lights(12V switches) Control
- Control via JSON Commands
- Supports Camera PT
- Supports RoArm-M2
- Control and Communicate via ESP-NOW

## Configure the compilation environment

Use vscode with PlatformIO IDE extension **platformio.platformio-ide** for development.

### Libraries

These [libraries](libs.md) are needed to build the project successfully. See [platformio.ini](platformio.ini) for details.

### Build

Simply run the **PlatformIO:Build** task.

#### Build options

| build flag         | default | description                          | flash size |
| ------------------ | ------- | ------------------------------------ | ---------- |
| ENABLE_GIMBAL      | 0       | use gimbal                           | 2.9 kB     |
| ENABLE_HTTP_SERVER | 0       | use http server to control the robot | 92.5 kB    |

### Upload firmware to target

Connect develop machine to the Rover via USB-C cable (marked with USB on the board) - than run the **PlatformIO:Upload** task.

### Basic Use

You can send JSON command to robot via UART/USB@115200 or Http Request/Web App.

To ensure compatibility with various types of robots. You can configure the robot by entering the following command:

    {"T":900,"main":2,"module":2}

In this command, the s directive denotes a robot-type setting. The first digit, `2`, signifies that the main type of robot is a `UGV Rover`, with `1` representing `RaspRover` and `3` indicating `UGV Beast`. The second digit, `2`, specifies the module as `Camera PT`, where `0` denotes `Nothing` and `1` signifies `RoArm-M2`.

# License

ugv_base_ros for the Waveshare UGV Robots: an open source robotics platform for the Robots based on **ROS Driver**.
Copyright (C) 2024 [Waveshare](https://www.waveshare.com/)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/gpl-3.0.txt>.
