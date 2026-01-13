// types.h

// Motor state tracking
struct MotorState {
    bool running;
    unsigned long stopTime;
    int pin;
    String id;
};
