// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_SERIAL_H
#define MICROROVER_SERIAL_H

#include <Arduino.h>     // Serial
#include <ArduinoJson.h> // JsonDocument, serializeJson

void sendSuccessResponse(String message) {
    JsonDocument response;
    response["success"] = true;
    response["message"] = message;

    serializeJson(response, Serial);
    Serial.println();
}

void sendErrorResponse(String message) {
    JsonDocument response;
    response["success"] = false;
    response["error"] = message;

    serializeJson(response, Serial);
    Serial.println();
}

#endif