# ESP32 Self-Balancing Robot

An experimental two-wheeled self-balancing robot built with an ESP32, MPU6050 IMU, L298N motor driver, and two geared DC motors.

> **Project status: Experimental prototype.** The robot responds to tilt and can balance briefly (approximately 6 seconds in recent tests), but does **not** yet balance reliably or remain stationary. This is not a finished autonomous balancing system.

## Features

- ESP32-based motor control
- MPU6050 angle estimation using a complementary filter
- PID-based balancing controller
- PWM motor output with startup/low-speed handling
- Fall detection and motor shutdown
- Serial diagnostics for tuning

## Hardware

- ESP32 DevKit V1
- MPU6050 (GY-521)
- L298N dual motor driver
- 2 × TT geared DC motors and wheels
- 2 × 18650 batteries
- XL4015 step-down converter
- DIY chassis

## Wiring

| Signal | ESP32 GPIO |
| --- | ---: |
| L298N ENA | 23 |
| L298N IN1 | 26 |
| L298N IN2 | 25 |
| L298N ENB | 32 |
| L298N IN3 | 33 |
| L298N IN4 | 14 |
| MPU6050 SDA | 21 |
| MPU6050 SCL | 22 |

Ensure a common ground between the controller, sensor, and motor driver. Verify power connections and voltage levels before operation.

## Firmware

The latest tested sketch is [`firmware/SelfBalancingRobot_Final_Test.ino`](firmware/SelfBalancingRobot_Final_Test.ino).

**Requirements:** Arduino IDE with ESP32 board support and the `MPU6050_light` library.

1. Open the `.ino` file in Arduino IDE.
2. Select your ESP32 board and serial port.
3. Install `MPU6050_light` if needed.
4. Upload the firmware.
5. Hold the robot upright and motionless during gyro calibration.
6. Test on a clear floor while ready to catch the robot.

## Controller

The sketch estimates body tilt from accelerometer and gyroscope measurements and computes a PID motor command. The latest tested baseline uses `Kp = 12`, `Ki = 100`, `Kd = 0.20`, and a target tilt of approximately `-1.7°`. These values are specific to this prototype, not general tuning recommendations.

## Testing and limitations

The prototype has demonstrated short balancing intervals of up to approximately **6 seconds** in recent tests. It can then accelerate forward or backward and fall. Multiple tuning attempts reduced motor jitter but did not achieve sustained balance.

Without wheel encoders, the controller has no direct wheel-speed or position feedback. Reliable stationary balancing is therefore not demonstrated. Mechanical stiffness, motor response, friction, and drivetrain characteristics may also affect performance.

## Potential improvements

- Add wheel encoders and an outer speed/position loop
- Improve chassis rigidity and mass distribution
- Characterize left/right motor response under load
- Revisit controller tuning after measuring wheel speed

## Media

Add a photo of the finished prototype to `images/` and, optionally, a short demonstration video link here.

## Project outcome

This project served as a practical exercise in embedded programming, I2C sensor integration, sensor fusion, PWM motor control, PID feedback, and experimental debugging.
