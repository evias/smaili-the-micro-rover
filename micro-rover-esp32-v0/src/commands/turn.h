// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_COMMANDS_TURN_H
#define MICROROVER_COMMANDS_TURN_H

#include <Arduino.h>     // String
#include <ArduinoJson.h> // JsonDocument
#include <ESP32Servo.h>  // Servo.write

#include "../serial.h"
#include "../types.h"

#define MIN_ANGLE 0
#define MAX_ANGLE 160
#define CENTER_ANGLE 70

/// @brief Implementation for turning servo motors with an angle.
/// @details Angle is set in degrees (0-180).
/// @param motor The configured servo motor.
/// @param angle An angle, the target angle for the servo motor.
/// @return A JsonDocument with a success flag.
JsonDocument TurnServo(ServoDevice &servo, unsigned int angle) {
    JsonDocument response;

    sendDebugMessage(String("Turning servo motor"));

    // Validate angle range
    if (angle < MIN_ANGLE) {
        angle = MIN_ANGLE;
    }
    else if (angle > MAX_ANGLE) {
        angle = MAX_ANGLE;
    }

    servo.motor.write(angle);

    response["success"] = true;
    return response;
}

#endif