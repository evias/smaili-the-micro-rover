// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_COMMANDS_STOP_H
#define MICROROVER_COMMANDS_STOP_H

#include <Arduino.h>     // digitalWrite, LOW
#include <ArduinoJson.h> // JsonDocument

#include "../serial.h"
#include "../types.h"

/// @brief Implementation for stopping DC motors.
/// @param motor The motor that must be stopped.
/// @return A JsonDocument with a success flag.
JsonDocument StopMotor(MotorDevice& motor) {
    JsonDocument response;

    String debugMsg = String("Stopping motor: ");
    debugMsg.concat(motor.side);
    sendDebugMessage(debugMsg);

    digitalWrite(motor.dev.pins[0], LOW);
    motor.running = false;
    motor.stopTime = 0;

    response["success"] = true;
    return response;
}

#endif