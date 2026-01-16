// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_COMMANDS_START_H
#define MICROROVER_COMMANDS_START_H

#include <Arduino.h>     // digitalWrite, HIGH, millis
#include <ArduinoJson.h> // JsonDocument
#include <vector>

#include "../types.h"

JsonDocument StartMotors(std::vector<MotorDevice> motors, unsigned long duration) {
    JsonDocument response;

    for (int i = 0, m = motors.size(); i < m; i++) {
        digitalWrite(motors[i].dev.pins[0], HIGH);
        motors[i].running = true;

        if (duration > 0) {
            motors[i].stopTime = millis() + duration;
        } else {
            motors[i].stopTime = 0;  // 0 means run indefinitely
        }
    }

    response["success"] = true;
    return response;
}

#endif