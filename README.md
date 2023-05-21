# Autonomous Arduino Cart
This repository contains the code for an autonomous obstacle-course robot. The robot is equipped with a color sensor to detect the color of the tape, an ultrasonic sensor to detect walls and obstacles, and a servo motor to control the movement of the robot.

This project was completed as a team, with members [Teodor Oprea](https://www.linkedin.com/in/teodor-oprea/) and [Nathan Wong](https://www.linkedin.com/in/nathan-wong-317604241/).

<p align="center"><img  src="https://raw.githubusercontent.com/Niravanaa/ArduinoCart/main/ArduinoCart.png" width="400"></p>
<p align="center"><b>Image of the Cart</b></p>

## Table of Contents
* [Installation](#installation)
* [Usage](#usage)
* [Code Description](#code-description)
* [Contributing](#contributing)

## Installation
To use this code, you need the following hardware components:

* Arduino UNO R3
* Color sensor (TCS230/TCS3200)
* Ultrasonic sensor (HC-SR04)
* Infrared (IR) sensor
* 2 Servo motors (SG90)
* IR Remote Control
* Robot chassis

1. Clone this repository to your local machine using https://github.com/Niravanaa/ArduinoCart.git.
2. Connect the hardware components to the Arduino board according to the pin configuration specified in the code.
3. Open the main.cpp file using the Arduino IDE.
4. Upload the code to the Arduino board using the IDE.
5. Power up the robot and test the functionality.

## Usage
The robot can operate in three different modes: remote control, wall-following, and tape-following.

### Remote Control Mode
In this mode, the robot can be controlled using an infrared (IR) remote control. The following buttons on the IR remote control can be used to control the robot:
* Up arrow: Move forward
* Down arrow: Move backward
* Left arrow: Turn left
* Right arrow: Turn right
* Mode 1: Switch to wall-following mode
* Mode 2: Switch to tape-following mode
* Mode 3: Switch back to remote control mode

### Wall-Following Mode
In this mode, the robot follows a wall while avoiding obstacles. The robot will turn left or right depending on the distance between the robot and the wall. If there is no wall detected for a certain period, the robot will stop moving.

### Tape-Following Mode
In this mode, the robot follows a colored tape. The color of the tape can be set using the TapeColor enum in the code. The robot will turn left or right depending on the color of the tape. 

## Code Description
The code is written in C++ using the Arduino IDE. The main code is contained in the main.cpp file. The code is organized into namespaces for servo movement and sensor readings. The code also contains constants for pin configuration, color frequencies, distances, and delays. The modes and tape colors are defined using enums.

## Contributing
Contributions are welcome! If you find a bug or have an enhancement in mind, please open an issue or submit a pull request.
