// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_MICRO_ROVER_H
#define MICROROVER_MICRO_ROVER_H

#include <Arduino.h> // String
#include "types.h"

// Forward definition needed for MicroRover::OnLoop().
JsonDocument StopMotor(MotorDevice&);

/// @brief MicroRover describes a tiny smiling rover named Smaili!
/// @details Configures the MicroRover instance hardware mappings.
class MicroRover {
    bool          online_;
    String          name_;
    String       version_;
    bool       has_servo_;
    bool      has_sensor_;

    MotorDevice lft_motor_;
    MotorDevice rht_motor_;
    ServoDevice     servo_;
    SensorDevice   sensor_;

public:
    MicroRover(const char*, const char*);
    ~MicroRover() {}

    void Usage(); // Hello, world!
    void Setup(); // maps to sketch::setup()
    void OnLoop(); // maps to sketch::loop()

    void AddMotor(const char*, unsigned short, const char*);
    void SetServo(const char*, unsigned short);
    // Order of pins here is: TRIG (out), ECHO (in)
    void SetSensor(const char*, unsigned short, unsigned short);

    MotorDevice&  GetMotor(const char*);
    ServoDevice&  GetServo();
    SensorDevice& GetSensor();
    const String& GetName();
    const String& GetVersion();
    bool          IsOnline();

protected:
    MotorDevice& getMotorBySide(const char*);
};

#endif