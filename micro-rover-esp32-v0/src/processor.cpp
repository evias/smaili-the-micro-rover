// SPDX-License-Identifier: BSD-3-Clause
#include <Arduino.h>    // Serial, String, digitalWrite, pinMode
#include <ArduinoJson.h> // JsonDocument

#include "processor.h"
#include "serial.h"

#include "commands/status.h"
#include "commands/start.h"
#include "commands/turn.h"
#include "commands/scan.h"
#include "commands/stop.h"

Command Processor::Process(String json) {
    JsonDocument doc;

    if (json == "help" || json == "info" || json == "?") {
        // Will show usage
        doc["command"] = "help";
    } else {
        // Parse JSON
        DeserializationError error = deserializeJson(doc, json);
        if (error) {
            sendErrorResponse("Invalid JSON: " + String(error.c_str()));
        }
    }

    return doc;
}

void Processor::Handle(MicroRover *rover, Command cmd) {
    // Extract command
    const char* command = cmd["command"];
    JsonDocument response;

    if (command == nullptr) {
        sendErrorResponse("Missing 'command' field");
        return;
    }

    if (String(command) == String("status")) {
        response = Status(rover);
    }
    else if (String(command) == String("scan")) {
        unsigned int count = 1;
        unsigned int interval = 500;
        if (cmd.containsKey("options") && cmd["options"].containsKey("count")) {
            count = cmd["options"]["count"].as<unsigned int>();
        }
        if (cmd.containsKey("options") && cmd["options"].containsKey("interval")) {
            interval = cmd["options"]["interval"].as<unsigned int>();
        }

        response = Scan(rover->GetSensor(), count, interval);
    }
    else if (String(command) == String("turn")) {
        unsigned int angle = 0;
        if (cmd.containsKey("options") && cmd["options"].containsKey("angle")) {
            angle = cmd["options"]["angle"].as<unsigned int>();
        }

        response = TurnServo(rover->GetServo(), angle);
    }
    else if (String(command) == String("start")) {
        unsigned long duration = 0;
        if (cmd.containsKey("options") && cmd["options"].containsKey("duration")) {
            duration = cmd["options"]["duration"].as<unsigned long>();
        }

        const char* side = "left";
        if (cmd.containsKey("options") && cmd["options"].containsKey("side")) {
            side = cmd["options"]["side"].as<const char*>();
        }

        response = StartMotor(rover->GetMotor(side), duration);
    }
    else if (String(command) == String("stop")) {
        const char* side = "left";
        if (cmd.containsKey("options") && cmd["options"].containsKey("side")) {
            side = cmd["options"]["side"].as<const char*>();
        }

        response = StopMotor(rover->GetMotor(side));
    }
    else {
        rover->Usage();
    }

    if (response.containsKey("success")) {
        serializeJson(response, Serial);
        Serial.println();
    }
}
