// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_SERIAL_H
#define MICROROVER_SERIAL_H

#include <Arduino.h>     // Serial
#include <ArduinoJson.h> // JsonDocument, serializeJson

/// @brief Prepare and send a success response JSON on serial port.
/// @param message The message to attach to the response.
inline void sendSuccessResponse(String message) {
    JsonDocument response;
    response["success"] = true;
    response["message"] = message;

    serializeJson(response, Serial);
    Serial.println();
}

/// @brief Prepare and send an error response JSON on serial port.
/// @param message The error to attach to the response.
inline void sendErrorResponse(String message) {
    JsonDocument response;
    response["success"] = false;
    response["error"] = message;

    serializeJson(response, Serial);
    Serial.println();
}

#endif