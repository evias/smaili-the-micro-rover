// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_COMMANDS_SCAN_H
#define MICROROVER_COMMANDS_SCAN_H

#include <Arduino.h>     // digitalWrite, delay
#include <ArduinoJson.h> // JsonDocument
#include <string>

#include "../serial.h"
#include "../types.h"
#include "../micro_rover.h"

// Speed of sound in cm/microsecond
#define SOUND_SPEED_CM_PER_MS 0.034

// Maximum distance to measure in cm
#define MAX_DISTANCE_CM 400

// Timeout for ultrasonic reading (30ms)
#define USONIC_TIMEOUT_MS 30000

/// @brief Implementation for ultrasonic sensor (HC-SR04) readings.
/// @param sensor The configured ultrasonic sensor device.
/// @param count The number of readings to execute.
/// @param interval Introduce a wait time between multiple readings.
/// @return A JsonDocument with distances per reading, starts with reading_1.
JsonDocument Scan(SensorDevice& sensor, unsigned int count, unsigned int interval) {
    JsonDocument response;

    for (int i = 0; i < count; i++) {
        sendDebugMessage(String("Reading from ultrasonic sensor"));

        // Clear the trigger pin
        digitalWrite(sensor.dev.pins[0], LOW);
        delayMicroseconds(2);

        String reading_id = "reading_";
        reading_id.concat(std::to_string(i+1).c_str());

        // Send 10 microsecond pulse to trigger
        digitalWrite(sensor.dev.pins[0], HIGH);
        delayMicroseconds(10);
        digitalWrite(sensor.dev.pins[0], LOW);

        // Read the echo pin, returns pulse duration in microseconds
        long duration = pulseIn(sensor.dev.pins[1], HIGH, USONIC_TIMEOUT_MS);

        // Calculate distance in centimeters
        // Distance = (Time x Speed of Sound) / 2
        float distance = (duration * SOUND_SPEED_CM_PER_MS) / 2.0;

        // Check for out of range or no echo
        if (duration == 0 || distance > MAX_DISTANCE_CM || distance <= 0) {
            response[reading_id] = String("NO ECHO");
        } else {
            response[reading_id] = distance;
        }

        sendDebugMessage(String("Done reading from ultrasonic sensor"));

        // Delay next reading
        if (i < count-1 && interval > 0) {
            delay(interval);
        }
    }

    response["success"] = true;
    return response;
}

#endif