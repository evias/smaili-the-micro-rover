// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_TYPES_H
#define MICROROVER_TYPES_H

#include <Arduino.h>     // String
#include <ArduinoJson.h> // JsonDocument
#include <ESP32Servo.h>  // Servo

typedef JsonDocument Command;

/// @brief  Device describes a device wired with pins to a ESP32 board.
/// @tparam NUM_PINS 
template <unsigned int NUM_PINS>
struct Device {
    String         id;
    unsigned short pins[NUM_PINS];
};

/// @brief MotorDevice describes a DC motor wired with 1 pin.
struct MotorDevice {
    Device<1>          dev; // 0=VCC
    bool           running;
    String            side;
    unsigned long stopTime;
};

/// @brief ServoDevice describes a Servo motor wired with 1 pin.
struct ServoDevice {
    Device<1>        dev; // 0=PWM
    Servo          motor;
    unsigned int   angle;
};

/// @brief SensorDevice describes a Ultrasonic sensor wired with 2 pins.
struct SensorDevice {
    Device<2> dev; // 0=TRIG, 1=ECHO
};

#endif