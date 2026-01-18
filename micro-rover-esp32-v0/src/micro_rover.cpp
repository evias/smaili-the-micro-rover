// SPDX-License-Identifier: BSD-3-Clause
#include <Arduino.h>    // Serial, String, digitalWrite, pinMode
#include <ESP32Servo.h> // ESP32PWM, Servo

#include "micro_rover.h"
#include "serial.h"

/// @brief MicroRover public constructor, initializes ESP32PWM timers.
/// @param name A friendly name for your MicroRover, e.g. "Smaili".
/// @param version A version number, recommended semantic versioning, e.g. "1.0.0".
MicroRover::MicroRover(const char* name, const char* version)
    : name_(String(name)),
      version_(String(version)),
      online_(false),
      has_sensor_(false),
      has_servo_(false)
{
    // Timer 0: Servo at 50Hz
    ESP32PWM::allocateTimer(0);

    // Timer 1: DC motors at diff freq
    ESP32PWM::allocateTimer(1);

    // Timer 2, 3 reserved for future use.
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    sendDebugMessage(String("MicroRover initialized"));
}

/// @brief The Usage method prints usage informations for this MicroRover.
void MicroRover::Usage() 
{
    Serial.println();
    Serial.println("===========================================");
    Serial.printf("  %s v%s: MicroRover Commands\n", name_.c_str(), version_.c_str());
    Serial.println("===========================================");
    Serial.println();
    Serial.println("Query status:");
    Serial.println("  {\"command\":\"status\"}");
    Serial.println("Start motor(s):");
    Serial.println("  {\"command\":\"start\",\"options\":{\"side\":\"right\",\"duration\":1200}}");
    Serial.println("Stop motor(s):");
    Serial.println("  {\"command\":\"stop\",\"options\":{\"side\":\"left\"}}");
    Serial.println("Turn servo to angle (10-150):");
    Serial.println("  {\"command\":\"turn\",\"options\":{\"angle\":80}}");
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
    pinMode(lft_motor_.dev.pins[0], OUTPUT);
    pinMode(rht_motor_.dev.pins[0], OUTPUT);
    digitalWrite(lft_motor_.dev.pins[0], LOW);
    digitalWrite(rht_motor_.dev.pins[0], LOW);

    // Configure ultrasonic sensor pins
    if (has_sensor_) {
        pinMode(sensor_.dev.pins[0], OUTPUT);
        pinMode(sensor_.dev.pins[1], INPUT);
        digitalWrite(sensor_.dev.pins[0], LOW);
    }

    // Configure servo motor pins
    if (has_servo_) {
        servo_.motor.setPeriodHertz(50);  // Standard 50Hz servo
        servo_.motor.attach(servo_.dev.pins[0], 500, 2400);  // Min/max pulse width
        servo_.motor.write(servo_.angle);
    }

    sendDebugMessage(String("MicroRover setup completed"));
}

/// @brief The OnLoop method verifies the expiration of motor runtimes.
/// @details This method is called from the sketch's loop() function.
void MicroRover::OnLoop() {
    unsigned long currentTime = millis();

    if (lft_motor_.running && lft_motor_.stopTime > 0 &&
        currentTime >= lft_motor_.stopTime) {
        StopMotor(this->lft_motor_);
    }

    if (rht_motor_.running && rht_motor_.stopTime > 0 &&
        currentTime >= rht_motor_.stopTime) {
        StopMotor(this->rht_motor_);
    }
}

/// @brief The AddMotor method registers a motor with a pin and a side of the MicroRover.
/// @param id A name for the component, e.g. "my-motor-1".
/// @param pin A pin number, corresponding to the ESP32 pin number, e.g. 2 for D2.
/// @param side A side of the MicroRover, one of "right" or "left".
void MicroRover::AddMotor(const char* id, unsigned short pin, const char* side) {
    if (String(side) == String("left")) {
        lft_motor_ = MotorDevice{Device<1>{String(id), {pin}}, false, String(side), 0};
        sendDebugMessage(String("Registered left-side DC motor"));
    } else if (String(side) == String("right")) {
        rht_motor_ = MotorDevice{Device<1>{String(id), {pin}}, false, String(side), 0};
        sendDebugMessage(String("Registered right-side DC motor"));
    } else {
        sendErrorResponse(String("Invalid wheel side. Please use 'left' or 'right'."));
        return ;
    }
}

/// @brief The SetServo method registers a servo motor with a pin.
/// @param id A name for the component, e.g. "my-servo".
/// @param pin A pin number, corresponding to the ESP32 pin number, e.g. 5 for D5.
void MicroRover::SetServo(const char* id, unsigned short pin) {
    Servo motor;
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

/// @brief The GetMotor method returns a MotorDevice instance.
/// @param side A side of the MicroRover, one of "right" or "left".
/// @return The initialized motor device instance.
MotorDevice& MicroRover::GetMotor(const char* side) {
    return getMotorBySide(side);
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

/// @brief The GetName method returns the MicroRover name.
/// @return The name of the MicroRover instance.
const String& MicroRover::GetName() {
    return this->name_;
}

/// @brief The GetVersion method returns the MicroRover version.
/// @return The version of the MicroRover instance.
const String& MicroRover::GetVersion() {
    return this->version_;
}

/// @brief The IsOnline method returns whether the MicroRover is connected to WiFi.
/// @return Returns true if the MicroRover is connected to a WiFi network.
bool MicroRover::IsOnline() {
    return this->online_;
}

/// @brief The getMotorBySide method returns a MotorDevice instance.
/// @param side A side of the MicroRover, one of "right" or "left".
/// @return Returns a MotorDevice instance.
MotorDevice& MicroRover::getMotorBySide(const char* side) {
    if (String(side) == String("left")) {
        return this->lft_motor_;
    }

    return this->rht_motor_;
}
