// SPDX-License-Identifier: BSD-3-Clause
#include <Arduino.h>    // Serial, String, digitalWrite, pinMode
#include <ESP32Servo.h> // ESP32PWM, Servo

#include "micro_rover.h"
#include "serial.h"

/// @brief MicroRover public constructor, initializes ESP32PWM timers.
/// @param name A friendly name for your MicroRover, e.g. "Smailì".
/// @param version A version number, recommended semantic versioning, e.g. "1.0.0".
MicroRover::MicroRover(const char* name, const char* version)
    : name_(name),
      version_(version),
      online_(false),
      has_sensor_(false),
      has_servo_(false),
      motors_({})
{
    // Timer 0: Servo at 50Hz
    ESP32PWM::allocateTimer(0);

    // Timer 1: DC motors at diff freq
    ESP32PWM::allocateTimer(1);

    // Timer 2, 3 reserved for future use.
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
}

/// @brief The Usage method prints usage informations for this MicroRover.
void MicroRover::Usage() 
{
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

/// @brief The Setup method initializes configured pin numbers.
/// @details This method is called from the sketch's setup() function.
void MicroRover::Setup()
{
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

/// @brief The OnLoop method verifies the expiration of motor runtimes.
/// @details This method is called from the sketch's loop() function.
void MicroRover::OnLoop() {
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

/// @brief The AddMotor method registers a motor with a pin and a side of the MicroRover.
/// @param id A name for the component, e.g. "my-motor-1".
/// @param pin A pin number, corresponding to the ESP32 pin number, e.g. 2 for D2.
/// @param side A side of the MicroRover, one of "right" or "left".
void MicroRover::AddMotor(const char* id, unsigned short pin, const char* side) {
    if (String(side) == String("both")) {
        sendErrorResponse(String("wheel side 'both' cannot be used in setup"));
        return ;
    }

    motors_.push_back(MotorDevice{Device<1>{String(id), {pin}}, false, String(side), 0});
}

/// @brief The SetServo method registers a servo motor with a pin.
/// @param id A name for the component, e.g. "my-servo".
/// @param pin A pin number, corresponding to the ESP32 pin number, e.g. 5 for D5.
void MicroRover::SetServo(const char* id, unsigned short pin) {
    // Configure the servo motor frequency
    Servo motor;
    motor.setPeriodHertz(50);  // Standard 50Hz servo
    motor.attach(pin, 500, 2400);  // Min/max pulse width

    servo_ = ServoDevice{Device<1>{String(id), {pin}}, motor, 80}; // angle=80
    has_servo_ = true;
}

/// @brief The SetSensor method registers a ultrasonic sensor with TRIG and ECHO pins.
/// @param id A name for the component, e.g. "my-usonic-sensor".
/// @param pin_out A pin number, i.e. to the ESP32 pin number wired to TRIG (out).
/// @param pin_in A pin number, i.e. to the ESP32 pin number wired to ECHO (in).
void MicroRover::SetSensor(const char* id, unsigned short pin_out, unsigned short pin_in) {
    sensor_ = SensorDevice{Device<2>{String(id), {pin_out, pin_in}}};
    has_sensor_ = true;
}

/// @brief The GetMotors method returns a vector of motors by side.
/// @param side A side of the MicroRover, one of "both", "right" or "left".
/// @return Returns a vector of MotorDevice instances filtered by side.
std::vector<MotorDevice> MicroRover::GetMotors(const char* side) {
    if (String(side) == String("both")) {
        return motors_;
    }

    return getMotorsBySide(side);
}

/// @brief The GetServo method returns a ServoDevice instance.
/// @return The initialized servo motor instance.
ServoDevice& MicroRover::GetServo() {
    return this->servo_;
}

/// @brief The GetSensor method returns a SensorDevice instance.
/// @return The initialized ultrasonic sensor instance.
SensorDevice& MicroRover::GetSensor() {
    return this->sensor_;
}

/// @brief The getMotorsBySide method returns a vector of motors by side.
/// @param side A side of the MicroRover, one of "both", "right" or "left".
/// @return Returns a vector of MotorDevice instances filtered by side.
/// @todo Should return reference to MotorDevice instances.
std::vector<MotorDevice> MicroRover::getMotorsBySide(const char* side) {
    std::vector<MotorDevice> out({});
    for (const MotorDevice& motor : motors_) {
        if (String(side) == String("both") || motor.side == String(side)) {
            out.push_back(motor);
        }
    }

    return out;
}
