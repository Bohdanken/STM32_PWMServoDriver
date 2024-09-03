# STM32_PWMServoDriver Library

![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)

## Overview

`STM32_PWMServoDriver` is a library designed for controlling a 16-channel PWM & Servo driver using the PCA9685 for STM32 microcontrollers. The library leverages the STM32 HAL I2C functions to communicate with the PCA9685 via I2C, requiring only two pins. This driver is particularly useful for projects requiring multiple PWM signals such as controlling servos or LEDs.

## Features

- **16 PWM Channels**: Control up to 16 independent PWM channels.
- **I2C Communication**: Uses I2C for communication with the PCA9685.
- **PWM Frequency Control**: Set the PWM frequency for the entire chip.
- **Sleep/Wake Mode**: Put the device to sleep to save power and wake it up when needed.
- **Support for External Clock**: Optionally use an external clock for more precise control.
- **Servo Control**: Simple functions to control servos using microseconds.

## Getting Started

### Prerequisites

- STM32CubeMX configured project.
- STM32 HAL library included in your project.
- I2C configured in your STM32 project.

### Installation

To use this library in your STM32 project:

1. Download or clone this repository.
2. Include `STM32_PWMServoDriver.h` and `STM32_PWMServoDriver.cpp` in your project.
3. Include the header file in your source code:
   ```cpp
   #include "STM32_PWMServoDriver.h"
