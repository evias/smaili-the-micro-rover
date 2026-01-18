// SPDX-License-Identifier: BSD-3-Clause
/**
 * Smaili the MicroRover: A tiny *smiling* (:smiley:) rover that speaks JSON!
 *
 * Example commands (send over Serial port):
 * - `{"command":"status"}`
 * - `{"command":"start","options":{"side":"right","duration":3000}}`
 * - `{"command":"start","options":{"side":"left","duration":3000}}`
 * - `{"command":"turn","options":{"angle":10}}`
 * - `{"command":"turn","options":{"angle":140}}`
 * - `{"command":"scan","options":{"count":5,"interval":500}}
 */

#include <Arduino.h> // Serial, String

#include "src/wiring.h"
#include "src/micro_rover.h"
#include "src/processor.h"

const char* VERSION = "1.0.0";
MicroRover* SMAILI_ROVER;
Processor*   SMAILI_PROC;

void setup() {
    Serial.begin(9600);

    SMAILI_ROVER = new MicroRover("Smaili", VERSION);
    SMAILI_ROVER->AddMotor("wheels-right", MOTOR_A_PIN, "right");
    SMAILI_ROVER->AddMotor("wheels-left", MOTOR_B_PIN, "left");
    SMAILI_ROVER->SetServo("servo", SERVO_SG90_PIN);
    SMAILI_ROVER->SetSensor("sensor", USONIC_TRIG_PIN, USONIC_ECHO_PIN);

    SMAILI_ROVER->Setup();
    SMAILI_ROVER->Usage();

    SMAILI_PROC = new Processor();

    delay(2000);
    Serial.println();
    Serial.println("MicroRover is now listening to your JSON whispers...");
}

void loop() {
    // Check for incoming Serial data
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() > 0) {
            Command cmd = SMAILI_PROC->Process(input);
            SMAILI_PROC->Handle(SMAILI_ROVER, cmd);
        }
    }

    SMAILI_ROVER->OnLoop();
}
