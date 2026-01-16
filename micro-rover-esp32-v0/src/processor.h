// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_PROCESSOR_H
#define MICROROVER_PROCESSOR_H

#include <Arduino.h>     // String, Serial
#include <ArduinoJson.h> // JsonDocument

#include "types.h"
#include "micro_rover.h"

#include "commands/status.h"
#include "commands/start.h"
#include "commands/stop.h"
#include "commands/turn.h"
#include "commands/scan.h"

class Processor {
public:
    Processor() {}

    Command Process(String json) {
        // Parse JSON
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, json);

        if (error) {
            sendErrorResponse("Invalid JSON: " + String(error.c_str()));
        }

        return doc;
    }

    void Handle(MicroRover *rover, Command cmd) {
        // Extract command
        const char* command = cmd["command"];
        JsonDocument response;

        if (command == nullptr) {
            sendErrorResponse("Missing 'command' field");
            return;
        }

        if (command == "status") {
            response = Status(rover);
        }
        else if (command == "scan") {
            unsigned int count = 1;
            unsigned int interval = 500;
            if (cmd.containsKey("options") && cmd["options"].containsKey("count")) {
                count = cmd["options"]["count"].as<unsigned int>();
            }
            if (cmd.containsKey("options") && cmd["options"].containsKey("interval")) {
                count = cmd["options"]["interval"].as<unsigned int>();
            }

            response = Scan(rover->GetSensor(), count, interval);
        }
        else if (command == "turn") {
            unsigned short angle = 0;
            if (cmd.containsKey("options") && cmd["options"].containsKey("angle")) {
                angle = cmd["options"]["angle"].as<unsigned short>();
            }

            response = TurnServo(rover->GetServo(), angle);
        }
        else if (command == "start") {
            unsigned long duration = 0;
            if (cmd.containsKey("options") && cmd["options"].containsKey("duration")) {
                duration = cmd["options"]["duration"].as<unsigned long>();
            }

            const char* side = "both";
            if (cmd.containsKey("options") && cmd["options"].containsKey("side")) {
                side = cmd["options"]["side"].as<const char*>();
            }

            std::vector<MotorDevice> motors = rover->GetMotors(side);
            response = StartMotors(motors, duration);
        }
        else if (command == "stop") {
            const char* side = "both";
            if (cmd.containsKey("options") && cmd["options"].containsKey("side")) {
                side = cmd["options"]["side"].as<const char*>();
            }

            std::vector<MotorDevice> motors = rover->GetMotors(side);
            response = StopMotors(motors);
        }

        serializeJson(response, Serial);
        Serial.println();
    }
};

#endif