// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_COMMANDS_TURN_H
#define MICROROVER_COMMANDS_TURN_H

#include <Arduino.h>     // String
#include <ArduinoJson.h> // JsonDocument
#include <ESP32Servo.h>  // Servo.write

#include "../types.h"

// Minimum angle in degrees (0-180), note that I use a custom
// "center" of 80 degrees... Normally, you'd set 90 there.

#define MIN_ANGLE 10
#define MAX_ANGLE 150
#define CENTER_ANGLE 80

JsonDocument TurnServo(ServoDevice &motor, unsigned short angle) {
    JsonDocument response;

    // Validate angle range
    if (angle < MIN_ANGLE || angle > MAX_ANGLE) {
        response["success"] = false;
        response["message"] = String("Angle must be between 10 and 150 degrees.");
        return response;
    }

    motor.servo.write(angle);

    response["success"] = true;
    return response;
}

#endif