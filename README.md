# The ROS2 stack for the Tinkerkit Braccio robot

## Overview

This repository contains the launch and configuration setups to get started with the [Tinkerkit Braccio][1] robot using ROS2, Gazebo and the driver to control the real robot. The Braccio is a fully operational robotic arm, featured with 5 servomotors plus one additional servomotor to control the gripper, and controller via an Arduino. Even its simplicity and low cost, it represents a perfect base to get use to ROS2 and to control a real robot arm, while testing new robot functionalities (ros_control, vision systems, etc).

The repository consists on following packages:

* *braccio bringup*. Main launch and bringup files.
* *braccio_description*. Robot decription and URDF files.
* *braccio_hardware*. Hardware interfaces for ROS-Arduino communication.
* *braccio_moveit_config*. Moveit2 integration.
* *braccio_ROS_Arduino*. Arduino library to connect with robot.

### braccio_bringup

Contains main launch files to run robot control on Gazebo Classic simulation or using the real robot.

### braccio_description

Braccio URDF files that completely describes the robot model. It also contains an RViz launch file to see robot model and allowed movements.

### braccio_hardware

All files that implement the hardware interface are in this folder. It is based on USB serial communication to send messages to the Arduino platform in order to control the robot. A bidirectional communicarion has been implemented in order to the ROS side to properly work, altough robot servo motors do not provide feedback to the controller: the Arduino directly copies the commands to state (or feedback) messages.

### braccio_moveit_config

MoveIt package that contains all required files to properly control robot arm and gripper. By default, two controllers have been implemented and tested:

1. *position_trajectory_controller*

* Type: [Joint trayectory controller][2].
* Command interface: position.
* State interface: position + velocity.

2. *gripper_controller*

* Type: [Joint trayectory controller][2].
* Command interface: position.
* State interface: position + velocity.

### braccio_ROS_Arduino

Contains the library to implement the hardware interface on the robot side, based on serial communication. In order to send commands to the braccio robot, an ad-hoc version of the ROSBraccio library has been included. So that, it is not necessary to install ROSBraccio library using the [Arduino IDE][3], but servo libraries might be required. Use the Arduino IDE to install it (refer to the [Arduino documentation][4] for its installation).

In order to properly control task execution, a task scheduler has been implemented, based on the [TaskScheduler library][5] (installation required).

* *T1*: Reads from serial.
* *T2*: Parses received message.
* *T3*: Sends commands to braccio robot.
* *T4*: Sends robot state to ROS (feedback).
* *T5*: Sends signal for ROS to check that Arduino is alive.

## Usage

### Model view

Launches RViz to show robot model.

     ros2 launch braccio_description view_robot.launch.py

### Gazebo classic simulation

Simulates the Braccio robot using Gazebo Classic and launches RViz to control it.

     ros2 launch braccio_bringup bringup.launch.py sim:=true

### Real robot control

Communicates with the Braccio robot via serial communication and launches RViz to control it.

     ros2 launch braccio_bringup bringup.launch.py sim:=true

### Testing communications

In order to test communication with the Arduino, a verbose option is implemented to check serial communication messages.

    ros2 launch braccio_bringup bringup.launch.py sim:=true hw_test:=true

or

     ros2 launch braccio_bringup bringup.launch.py sim:=false hw_test:=true


## Disclaimer

This repository contains packages under development to control the robot arm. Any use of this repository in environments with a real robot must be performed under strict caution to avoid damages to the robot itself and/or injuries to the operators.


[1]: https://store.arduino.cc/en-es/products/tinkerkit-braccio-robot?gad_source=1&gclid=Cj0KCQjwrp-3BhDgARIsAEWJ6SzPdT7gC3EQNAJt74VFt3OlSA6UL32x-Dm_4-oCrWqlIVi4uy5s2xsaAqwvEALw_wcB

[2]: https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html

[3]: https://www.arduino.cc/en/software

[4]: https://docs.arduino.cc/software/ide-v2/tutorials/ide-v2-installing-a-library/

[5]: https://github.com/arkhipenko/TaskScheduler

