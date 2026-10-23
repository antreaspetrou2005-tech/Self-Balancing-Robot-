# ESP32 Self-Balancing Robot — Prototype

## Overview

An experimental two-wheeled self-balancing robot developed using an ESP32 microcontroller, an MPU6050 inertial measurement unit (IMU), and an L298N motor driver.

The project focuses on hardware integration, sensor interfacing, motor control, and the development of a feedback-based balancing system.

**Project Status: Work in Progress**

The hardware prototype has been assembled and tested. An initial PID-based control algorithm has been implemented, but reliable autonomous balancing has not yet been achieved.

## Hardware Components

- ESP32 DevKit V1
- MPU6050 (GY-521) IMU
- L298N dual motor driver
- Two DC geared motors with wheels
- Two 18650 lithium-ion batteries
- XL4015 DC-DC buck converter
- Custom-built prototype chassis

## Implemented Features

- MPU6050 sensor communication via I2C
- Gyroscope and accelerometer measurements
- Tilt angle estimation using a complementary filter
- Bidirectional motor control using PWM
- Experimental PID balancing controller
- Safety cutoff for excessive tilt angles
- Serial monitoring for debugging and controller tuning

## Current Limitations

The robot responds to changes in tilt angle but cannot yet maintain stable, sustained autonomous balance. Further improvements to the control algorithm, motor response, and mechanical design are required.

## Future Development

- Improve balancing stability and controller tuning
- Refine motor control at low speeds
- Improve mechanical rigidity and component mounting
- Investigate wheel-speed feedback for improved motion control

## Project Goal

To gain practical experience in embedded systems, sensor fusion, feedback control, electronics integration, and iterative prototyping.
