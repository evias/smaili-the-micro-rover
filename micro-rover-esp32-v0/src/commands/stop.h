// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_COMMANDS_STOP_H
#define MICROROVER_COMMANDS_STOP_H

#include <Arduino.h>     // digitalWrite, LOW
#include <ArduinoJson.h> // JsonDocument

#include "../types.h"

/// @brief Implementation for stopping DC motors.
/// @param motors The list of motors that must be stopped.
/// @return A JsonDocument with a success flag.
JsonDocument StopMotors(std::vector<MotorDevice> motors) {
    JsonDocument response;

    for (int i = 0, m = motors.size(); i < m; i++) {
        digitalWrite(motors[i].dev.pins[0], LOW);
        motors[i].running = false;
        motors[i].stopTime = 0;
    }

    response["success"] = true;
    return response;
}

#endif