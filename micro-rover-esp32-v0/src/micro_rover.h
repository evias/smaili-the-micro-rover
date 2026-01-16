// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_MICRO_ROVER_H
#define MICROROVER_MICRO_ROVER_H

#include <vector>
#include "types.h"

// Forward definition needed for MicroRover::OnLoop().
JsonDocument StopMotors(std::vector<MotorDevice>);

/// @brief MicroRover describes a tiny smiling rover named Smailì!
/// @details Configures the MicroRover instance hardware mappings.
class MicroRover {
    typedef std::vector<MotorDevice> MotorDevices;

    bool          online_;
    const char*     name_;
    const char*  version_;
    bool       has_servo_;
    bool      has_sensor_;

    MotorDevices motors_;
    ServoDevice   servo_;
    SensorDevice sensor_;

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

    MotorDevices  GetMotors(const char*);
    ServoDevice&  GetServo();
    SensorDevice& GetSensor();

protected:
    MotorDevices getMotorsBySide(const char*);
};

#endif