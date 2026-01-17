// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_COMMANDS_STATUS_H
#define MICROROVER_COMMANDS_STATUS_H

#include <ArduinoJson.h> // JsonDocument, JsonObject
#include <vector>

#include "../types.h"
#include "../micro_rover.h"

#include "scan.h"

/// @brief Implementation for a status command.
/// @param rover The MicroRover instance.
/// @return A JsonDocument with all status information.
JsonDocument Status(MicroRover *rover) {
    JsonDocument response;

    // Motors statuses
    JsonObject motors = response.createNestedObject("motors");
    std::vector<MotorDevice> rhtMotors = rover->GetMotors("right");
    for (const MotorDevice& motor: rhtMotors) {
        JsonObject motorStatus = motors.createNestedObject(motor.dev.id);
        motorStatus["running"] = motor.running;
        motorStatus["pin"] = motor.dev.pins[0];
        if (motor.running && motor.stopTime > 0) {
            long remaining = motor.stopTime - millis();
            motorStatus["remaining_ms"] = remaining > 0 ? remaining : 0;
        }
    }

    std::vector<MotorDevice> lftMotors = rover->GetMotors("left");
    for (const MotorDevice& motor: lftMotors) {
        JsonObject motorStatus = motors.createNestedObject(motor.dev.id);
        motorStatus["running"] = motor.running;
        motorStatus["pin"] = motor.dev.pins[0];
        if (motor.running && motor.stopTime > 0) {
            long remaining = motor.stopTime - millis();
            motorStatus["remaining_ms"] = remaining > 0 ? remaining : 0;
        }
    }

    // Servo status
    JsonObject servoStatus = response.createNestedObject("servo");
    ServoDevice servo = rover->GetServo();
    servoStatus["angle"] = servo.angle;
    servoStatus["pin"] = servo.dev.pins[0];

    // Ultrasonic sensor - take a reading
    SensorDevice sensor = rover->GetSensor();
    JsonDocument reading = Scan(sensor, 1, 0) ; // 1 reading
    JsonObject ultrasonicStatus = response.createNestedObject("ultrasonic");
    ultrasonicStatus["trig_pin"] = sensor.dev.pins[0];
    ultrasonicStatus["echo_pin"] = sensor.dev.pins[1];
    if (!reading["success"]) {
        ultrasonicStatus["distance_cm"] = nullptr;
    } else {
        ultrasonicStatus["distance_cm"] = reading["reading_1"];
    }

    response["success"] = true;
    return response;
}

#endif