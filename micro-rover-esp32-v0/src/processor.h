// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_PROCESSOR_H
#define MICROROVER_PROCESSOR_H

#include <Arduino.h> // String
#include "types.h"

// Forward definition needed for Processor::Handle().
class MicroRover;

/// @brief Processor describes a JSON commands processor.
/// @details Accepts JSON commands such as "status", "start", "turn" and "scan".
class Processor {
public:
    Processor() {}

    Command Process(String);
    void    Handle(MicroRover *, Command);
};

#endif