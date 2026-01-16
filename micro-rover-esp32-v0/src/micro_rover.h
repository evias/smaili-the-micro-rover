// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_MICRO_ROVER_H
#define MICROROVER_MICRO_ROVER_H

#include <Arduino.h>    // Serial, String, digitalWrite, pinMode
#include <ESP32Servo.h> // ESP32PWM, Servo
#include <vector>

#include "types.h"
#include "serial.h"

#include "commands/stop.h"

class MicroRover {
    bool          online_;
    const char*     name_;
    const char*  version_;
    bool       has_servo_;
    bool      has_sensor_;

    std::vector<MotorDevice> motors_;
    ServoDevice   servo_;
    SensorDevice sensor_;

public:
    MicroRover(const char* name, const char* version)
        : name_(name), version_(version)
    {
        // Timer 0: Servo at 50Hz
        ESP32PWM::allocateTimer(0);

        // Timer 1: DC motors at diff freq
        ESP32PWM::allocateTimer(1);

        // Timer 2, 3 reserved for future use.
        ESP32PWM::allocateTimer(2);
        ESP32PWM::allocateTimer(3);
    }

    ~MicroRover() {}

    void Usage() {
        Serial.println();
        Serial.println("===========================================");
        Serial.println(sprintf("  %s v%s: MicroRover Commands", name_, version_));
        Serial.println("===========================================");
        Serial.println();
        Serial.println("Query status:");
        Serial.println("  {\"command\":\"status\"}");
        Serial.println();
        Serial.println("Start motor(s):");
        Serial.println("  {\"command\":\"start\",\"options\":{\"side\":\"right\",\"duration\":1200}}");
        Serial.println();
        Serial.println("Stop motor(s):");
        Serial.println("  {\"command\":\"stop\",\"options\":{\"side\":\"left\"}}");
        Serial.println();
        Serial.println("Stop all motors:");
        Serial.println("  {\"command\":\"stop\",\"options\":{\"side\":\"both\"}}");
        Serial.println();
        Serial.println("Turn servo to angle (10-150):");
        Serial.println("  {\"command\":\"turn\",\"options\":{\"angle\":80}}");
        Serial.println();
        Serial.println("Scan distance with sensor (5 readings):");
        Serial.println("  {\"command\":\"scan\",\"options\":{\"count\":5,\"interval\":500}}");
        Serial.println();
        Serial.println("===========================================");
        Serial.println();
    }

    void Setup() {
        // Configure DC motors pins
        for (const MotorDevice& motor : motors_) {
            pinMode(motor.dev.pins[0], OUTPUT);
            digitalWrite(motor.dev.pins[0], LOW);
        }

        // Configure ultrasonic sensor pins
        if (has_sensor_) {
            pinMode(sensor_.dev.pins[0], OUTPUT);
            pinMode(sensor_.dev.pins[1], INPUT);
            digitalWrite(sensor_.dev.pins[0], LOW);
        }
    }

    void AddMotor(const char* id, unsigned short pin, const char* side) {
        if (String(side) == String("both")) {
            sendErrorResponse(String("wheel side both cannot be used in setup"));
            return ;
        }

        motors_.push_back(MotorDevice{Device<1>{String(id), {pin}}, false, String(side)});
    }

    void SetServo(const char* id, unsigned short pin) {
        // Configure the servo motor frequency
        Servo motor;
        motor.setPeriodHertz(50);  // Standard 50Hz servo
        motor.attach(pin, 500, 2400);  // Min/max pulse width

        servo_ = ServoDevice{Device<1>{String(id), {pin}}, motor};
        has_servo_ = true;
    }

    // Order of pins here is: TRIG (out), ECHO (in)
    void SetSensor(const char* id, unsigned short pin_out, unsigned short pin_in) {
        sensor_ = SensorDevice{Device<2>{String(id), {pin_out, pin_in}}};
        has_sensor_ = true;
    }

    void HandleTimeouts() {
        unsigned long currentTime = millis();

        std::vector<MotorDevice> expired;
        for (int i = 0, m = motors_.size(); i < m; i++) {
            if (!motors_[i].running || motors_[i].stopTime <= 0) {
                continue;
            }

            if (currentTime >= motors_[i].stopTime) {
                expired.push_back(motors_[i]);
            }
        }

        if (expired.size() > 0) {
            StopMotors(expired);
        }
    }

    std::vector<MotorDevice> GetMotors(const char* side) {
        if (String(side) == String("both")) {
            return motors_;
        }
        
        return getMotorsBySide(side);
    }

    ServoDevice& GetServo() {
        return this->servo_;
    }

    SensorDevice& GetSensor() {
        return this->sensor_;
    }

protected:
    std::vector<MotorDevice> getMotorsBySide(const char* side) {
        std::vector<MotorDevice> out;
        for (const MotorDevice& motor : motors_) {
            if (String(side) == String("both") || motor.side == String(side)) {
                out.push_back(motor);
            }
        }

        return out;
    }
};

#endif