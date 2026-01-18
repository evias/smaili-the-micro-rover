// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_COMMANDS_START_H
#define MICROROVER_COMMANDS_START_H

#include <Arduino.h>     // digitalWrite, HIGH, millis
#include <ArduinoJson.h> // JsonDocument

#include "../serial.h"
#include "../types.h"

/// @brief Implementation for starting DC motors.
/// @param motor The motor that must be started.
/// @param duration The number of milliseconds to keep motors running.
/// @return A JsonDocument with a success flag.
JsonDocument StartMotor(MotorDevice& motor, unsigned long duration) {
    JsonDocument response;

    String debugMsg = String("Starting motor: ");
    debugMsg.concat(motor.side);
    sendDebugMessage(debugMsg);

    digitalWrite(motor.dev.pins[0], HIGH);
    motor.running = true;

    if (duration > 0) {
        motor.stopTime = millis() + duration;
    } else {
        motor.stopTime = 0;  // 0 means run indefinitely
    }

    response["success"] = true;
    return response;
}

#endif