// SPDX-License-Identifier: BSD-3-Clause
#ifndef MICROROVER_WIRING_H
#define MICROROVER_WIRING_H

/**
 * DC Motors via Transistors
 *
 * DC(+) -> esp32(D2) -> Resistor(+) -> Diode(S)
 * Resistor(-) -> Transistor(B)
 * DC(-) -> Transistor(C) -> Diode(P)
 * esp32(GND) -> transistor(E)
 *
 * Motor_A refers to *rear-right-side* DC motor.
 * Motor_B refers to *rear-left-side* DC motor.
 **/
const unsigned short MOTOR_A_PIN = 2; // D2
const unsigned short MOTOR_B_PIN = 4; // D4

/**
 * Micro Servo SG90
 *
 * esp32(GND) -> SG(B)
 * esp32(3v3) -> SG(R)
 * esp32(D5) -> SG(Y)
 **/
const unsigned short SERVO_SG90_PIN  = 5; // D5

/**
 * Ultrasonic sensor HC-SR04
 *
 * esp32(3v3) -> HC(R)
 * esp32(D18) -> HC(G)
 * esp32(D15) -> HC(W)
 * esp32(GND) -> HC(B)
 **/
const unsigned short USONIC_TRIG_PIN = 18; // D18
const unsigned short USONIC_ECHO_PIN = 15; // D15

#endif