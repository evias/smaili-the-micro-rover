// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_TYPES_H
#define MICROROVER_TYPES_H

#include <Arduino.h>     // String
#include <ArduinoJson.h> // JsonDocument
#include <ESP32Servo.h>  // Servo

typedef JsonDocument Command;

template <unsigned int NUM_PINS>
struct Device {
    String         id;
    unsigned short pins[NUM_PINS];
};

struct MotorDevice {
    Device<1>          dev; // 0=VCC
    bool           running;
    String            side;
    unsigned long stopTime;
};

struct ServoDevice {
    Device<1>        dev; // 0=PWM
    Servo          servo;
    unsigned short angle;
};

struct SensorDevice {
    Device<2> dev; // 0=TRIG, 1=ECHO
};

#endif