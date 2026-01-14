// micro-rover-wiring-test.ino
// An arduino sketch for the micro-rover to test wiring of hardware components.
//
// Copyright 2026 Grégory Saive <greg@evi.as> for re:Software S.L. (resoftware.es).

#include <ESP32Servo.h>

// ============================================
// IMPORTANT: Adjust these pin numbers to match
// the actual labels printed on YOUR ESP32 board!
// ============================================

const int MOTOR_A_PIN = 2; // D2
const int MOTOR_B_PIN = 4; // D4
const int SERVO_PIN = 5;   // D5
const int ULTRASONIC_TRIG = 18; // D18
const int ULTRASONIC_ECHO = 15; // D15

Servo testServo;

void setup() {
    Serial.begin(9600);
    delay(2000);

    Serial.println();
    Serial.println("=========================================");
    Serial.println("  ESP32 Component Diagnostic Tool");
    Serial.println("=========================================");
    Serial.println();

    // Setup all pins
    pinMode(MOTOR_A_PIN, OUTPUT);
    pinMode(MOTOR_B_PIN, OUTPUT);
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);

    digitalWrite(MOTOR_A_PIN, LOW);
    digitalWrite(MOTOR_B_PIN, LOW);
    digitalWrite(ULTRASONIC_TRIG, LOW);

    // Setup servo
    ESP32PWM::allocateTimer(0);
    testServo.setPeriodHertz(50);
    testServo.attach(SERVO_PIN, 500, 2400);

    printMenu();
}

void printMenu() {
    Serial.println();
    Serial.println("--- DIAGNOSTIC MENU ---");
    Serial.println("Send a command:");
    Serial.println("  1 = Test Motor A (GPIO " + String(MOTOR_A_PIN) + ")");
    Serial.println("  2 = Test Motor B (GPIO " + String(MOTOR_B_PIN) + ")");
    Serial.println("  3 = Test Servo (GPIO " + String(SERVO_PIN) + ")");
    Serial.println("  4 = Test Ultrasonic Sensor");
    Serial.println("  5 = Test ALL GPIO outputs (find which pins work)");
    Serial.println("  6 = Read Ultrasonic continuously for 10 seconds");
    Serial.println("  7 = Show pin voltage states");
    Serial.println("  ? = Show this menu");
    Serial.println();
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();

        // Clear any remaining characters
        while (Serial.available()) Serial.read();

        switch (cmd) {
            case '1':
                testMotor("Motor A", MOTOR_A_PIN);
                break;
            case '2':
                testMotor("Motor B", MOTOR_B_PIN);
                break;
            case '3':
                testServoMotor();
                break;
            case '4':
                testUltrasonic();
                break;
            case '5':
                testAllGPIO();
                break;
            case '6':
                continuousUltrasonicTest();
                break;
            case '7':
                showPinStates();
                break;
            case '?':
                printMenu();
                break;
            default:
                if (cmd != '\n' && cmd != '\r') {
                    Serial.println("Unknown command. Send '?' for menu.");
                }
                break;
        }
    }
}

void testMotor(String name, int pin) {
    Serial.println();
    Serial.println("=== Testing " + name + " on GPIO " + String(pin) + " ===");

    Serial.println("Setting pin HIGH (motor should run)...");
    digitalWrite(pin, HIGH);
    Serial.println("Pin is now HIGH. Waiting 3 seconds...");
    Serial.println(">>> Does the motor spin? <<<");
    delay(3000);

    Serial.println("Setting pin LOW (motor should stop)...");
    digitalWrite(pin, LOW);
    Serial.println("Pin is now LOW.");

    Serial.println();
    Serial.println("If motor did NOT spin:");
    Serial.println("  1. Check transistor wiring (Base to resistor to GPIO)");
    Serial.println("  2. Check external power supply is ON");
    Serial.println("  3. Check common ground connection");
    Serial.println("  4. Try a different GPIO pin");
    Serial.println();
}

void testServoMotor() {
    Serial.println();
    Serial.println("=== Testing Servo on GPIO " + String(SERVO_PIN) + " ===");

    Serial.println("Moving to 270 degrees...");
    testServo.write(270);
    delay(3000);

    Serial.println("Moving to 180 degrees...");
    testServo.write(180);
    delay(3000);

    Serial.println("Moving to 90 degrees...");
    testServo.write(90);
    delay(3000);

    Serial.println("Returning to 180 degrees...");
    testServo.write(180);
    delay(500);

    Serial.println("Servo test complete!");
    Serial.println();
}

void testUltrasonic() {
    Serial.println();
    Serial.println("=== Testing Ultrasonic Sensor ===");
    Serial.println("TRIG pin: GPIO " + String(ULTRASONIC_TRIG));
    Serial.println("ECHO pin: GPIO " + String(ULTRASONIC_ECHO));
    Serial.println();

    Serial.println("Taking 5 readings...");
    Serial.println();

    for (int i = 0; i < 5; i++) {
        // Clear trigger
        digitalWrite(ULTRASONIC_TRIG, LOW);
        delayMicroseconds(2);

        // Send pulse
        digitalWrite(ULTRASONIC_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRASONIC_TRIG, LOW);

        // Read echo
        long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);

        Serial.print("  Reading ");
        Serial.print(i + 1);
        Serial.print(": Duration = ");
        Serial.print(duration);
        Serial.print(" us");

        if (duration == 0) {
            Serial.println(" [NO ECHO - Check wiring!]");
        } else {
            float distance = (duration * 0.034) / 2.0;
            Serial.print(", Distance = ");
            Serial.print(distance);
            Serial.println(" cm");
        }

        delay(500);
    }

    Serial.println();
    Serial.println("If all readings show 'NO ECHO':");
    Serial.println("  1. Check VCC is connected to 5V (not 3.3V)");
    Serial.println("  2. Check GND is connected");
    Serial.println("  3. TRIG and ECHO might be swapped - try swapping them");
    Serial.println("  4. Try different GPIO pins");
    Serial.println();
}

void continuousUltrasonicTest() {
    Serial.println();
    Serial.println("=== Continuous Ultrasonic Test (10 seconds) ===");
    Serial.println("Move your hand in front of the sensor...");
    Serial.println();

    unsigned long startTime = millis();

    while (millis() - startTime < 10000) {
        digitalWrite(ULTRASONIC_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(ULTRASONIC_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRASONIC_TRIG, LOW);

        long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);

        if (duration > 0) {
            float distance = (duration * 0.034) / 2.0;
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.println(" cm");
        } else {
            Serial.println("No echo");
        }

        delay(200);
    }

    Serial.println();
    Serial.println("Test complete.");
    Serial.println();
}

void testAllGPIO() {
    Serial.println();
    Serial.println("=== Testing Multiple GPIO Pins ===");
    Serial.println("This will pulse various GPIO pins.");
    Serial.println("Watch your motor or use an LED to see which pins work.");
    Serial.println();

    // Safe GPIO pins on ESP32 DEVKIT V1
    int testPins[] = {2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
    int numPins = sizeof(testPins) / sizeof(testPins[0]);

    for (int i = 0; i < numPins; i++) {
        int pin = testPins[i];

        Serial.print("Testing GPIO ");
        Serial.print(pin);
        Serial.print("... ");

        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
        delay(3000);
        digitalWrite(pin, LOW);

        Serial.println("Done (HIGH for 3.0s)");
        delay(200);
    }

    Serial.println();
    Serial.println("GPIO scan complete!");
    Serial.println("Note which GPIO number made your motor/LED activate.");
    Serial.println();

    // Restore original pin modes
    pinMode(MOTOR_A_PIN, OUTPUT);
    pinMode(MOTOR_B_PIN, OUTPUT);
    digitalWrite(MOTOR_A_PIN, LOW);
    digitalWrite(MOTOR_B_PIN, LOW);
}

void showPinStates() {
    Serial.println();
    Serial.println("=== Current Pin Configuration ===");
    Serial.println();
    Serial.println("Motor A:    GPIO " + String(MOTOR_A_PIN));
    Serial.println("Motor B:    GPIO " + String(MOTOR_B_PIN));
    Serial.println("Servo:      GPIO " + String(SERVO_PIN));
    Serial.println("Ultrasonic TRIG: GPIO " + String(ULTRASONIC_TRIG));
    Serial.println("Ultrasonic ECHO: GPIO " + String(ULTRASONIC_ECHO));
    Serial.println();
}