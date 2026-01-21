// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_COMMANDS_STATUS_H
#define MICROROVER_COMMANDS_STATUS_H

#include <ArduinoJson.h> // JsonDocument, JsonObject

#include "../serial.h"
#include "../types.h"
#include "../micro_rover.h"

#include "scan.h"

/// @brief Implementation for a status command.
/// @param rover The MicroRover instance.
/// @return A JsonDocument with all status information.
JsonDocument Status(MicroRover *rover) {
    JsonDocument response;

    sendDebugMessage(String("Querying MicroRover Status"));

    response["online"] = rover->IsOnline();
    response["ipaddr"] = rover->GetIPAddress();

    // Motors statuses
    JsonObject motors = response.createNestedObject("motors");

    MotorDevice rhtMotor = rover->GetMotor("right");
    JsonObject rhtStatus = motors.createNestedObject(rhtMotor.dev.id);
    rhtStatus["running"] = rhtMotor.running;
    rhtStatus["pin"] = rhtMotor.dev.pins[0];
    if (rhtMotor.running && rhtMotor.stopTime > 0) {
        long remaining = rhtMotor.stopTime - millis();
        rhtStatus["remaining_ms"] = remaining > 0 ? remaining : 0;
    }

    MotorDevice lftMotor = rover->GetMotor("left");
    JsonObject lftStatus = motors.createNestedObject(lftMotor.dev.id);
    lftStatus["running"] = lftMotor.running;
    lftStatus["pin"] = lftMotor.dev.pins[0];
    if (lftMotor.running && lftMotor.stopTime > 0) {
        long remaining = lftMotor.stopTime - millis();
        lftStatus["remaining_ms"] = remaining > 0 ? remaining : 0;
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