// micro-rover-esp32-v0.ino
// An arduino sketch for the micro-rover with DC motors and servo-motor
// based ultrasonic sensor.
//
// Copyright 2026 Grégory Saive <greg@evi.as> for re:Software S.L. (resoftware.es).

#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include "types.h"

// ============================================
// PIN DEFINITIONS
// ============================================

// DC Motors via transistors
const int MOTOR_A_PIN = 2; // D2
const int MOTOR_B_PIN = 4; // D4
const int SERVO_SG90_PIN  = 5; // D5
const int USONIC_TRIG_PIN = 18; // D18
const int USONIC_ECHO_PIN = 15; // D15

// ============================================
// CONSTANTS
// ============================================

#define SOUND_SPEED_CM_PER_MS 0.034  // Speed of sound in cm/microsecond
#define MAX_DISTANCE_CM 400          // Maximum distance to measure
#define USONIC_TIMEOUT_MS 30000  // Timeout for ultrasonic reading (30s)

// ============================================
// OBJECTS
// ============================================

Servo servoMotor;
MotorState rhtMotor = {false, 0, MOTOR_A_PIN, "motor-right"};
MotorState lftMotor = {false, 0, MOTOR_B_PIN, "motor-left"};

// ============================================
// STATE
// ============================================

int currentServoAngle = 90;  // Start at center position

// ============================================
// SETUP
// ============================================

void setup() {
    Serial.begin(9600);

    // Configure motor pins as outputs
    pinMode(MOTOR_A_PIN, OUTPUT);
    pinMode(MOTOR_B_PIN, OUTPUT);

    // Ensure motors are OFF at startup
    digitalWrite(MOTOR_A_PIN, LOW);
    digitalWrite(MOTOR_B_PIN, LOW);

    // Configure ultrasonic sensor pins
    pinMode(USONIC_TRIG_PIN, OUTPUT);
    pinMode(USONIC_ECHO_PIN, INPUT);
    digitalWrite(USONIC_TRIG_PIN, LOW);

    // Configure servo
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    servoMotor.setPeriodHertz(50);  // Standard 50Hz servo
    servoMotor.attach(SERVO_SG90_PIN, 500, 2400);  // Min/max pulse width
    //servoMotor.write(currentServoAngle);  // Move to center position

    // Done setting up
    printUsage();
}

void printUsage() {
    Serial.println();
    Serial.println("===========================================");
    Serial.println("  Motor, Servo & Sensor Control System");
    Serial.println("===========================================");
    Serial.println();
    Serial.println("Available JSON commands:");
    Serial.println();
    Serial.println("--- MOTOR COMMANDS ---");
    Serial.println("Start motor for duration:");
    Serial.println("  {\"id\":\"motor-left\",\"command\":\"start\",\"options\":{\"duration\":1200}}");
    Serial.println();
    Serial.println("Start motor indefinitely:");
    Serial.println("  {\"id\":\"motor-right\",\"command\":\"start\"}");
    Serial.println();
    Serial.println("Stop motor:");
    Serial.println("  {\"id\":\"motor-left\",\"command\":\"stop\"}");
    Serial.println();
    Serial.println("Stop all motors:");
    Serial.println("  {\"command\":\"stop-all\"}");
    Serial.println();
    Serial.println("--- SERVO COMMANDS ---");
    Serial.println("Turn servo to angle (0-180):");
    Serial.println("  {\"command\":\"turn\",\"options\":{\"angle\":90}}");
    Serial.println();
    Serial.println("Turn servo with speed control:");
    Serial.println("  {\"command\":\"turn\",\"options\":{\"angle\":45,\"speed\":10}}");
    Serial.println();
    Serial.println("--- SENSOR COMMANDS ---");
    Serial.println("Read distance from ultrasonic sensor:");
    Serial.println("  {\"command\":\"read-distance\"}");
    Serial.println();
    Serial.println("Continuous distance monitoring (5 readings):");
    Serial.println("  {\"command\":\"scan\",\"options\":{\"count\":5,\"interval\":500}}");
    Serial.println();
    Serial.println("--- SYSTEM COMMANDS ---");
    Serial.println("Query status:");
    Serial.println("  {\"command\":\"status\"}");
    Serial.println();
    Serial.println("===========================================");
    Serial.println();
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
    // Check for incoming Serial data
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() > 0) {
            processCommand(input);
        }
    }

    // Check if motors need to be stopped (duration elapsed)
    checkMotorTimers();
    delay(1000);
}

// ============================================
// COMMAND PROCESSOR
// ============================================

void processCommand(String jsonString) {
    // Parse JSON
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, jsonString);

    if (error) {
        sendErrorResponse("Invalid JSON: " + String(error.c_str()));
        return;
    }

    // Extract command
    const char* command = doc["command"];

    if (command == nullptr) {
        sendErrorResponse("Missing 'command' field");
        return;
    }

    // ---- GLOBAL COMMANDS ----
    if (strcmp(command, "stop-all") == 0) {
        stopAllMotors();
        sendSuccessResponse("All motors stopped");
        return;
    }

    if (strcmp(command, "status") == 0) {
        sendStatusResponse();
        return;
    }

    // ---- SERVO COMMANDS ----
    if (strcmp(command, "turn") == 0) {
        handleServoCommand(doc);
        return;
    }

    // ---- ULTRASONIC SENSOR COMMANDS ----
    if (strcmp(command, "read-distance") == 0) {
        handleReadDistanceCommand();
        return;
    }
    
    if (strcmp(command, "scan") == 0) {
        handleScanCommand(doc);
        return;
    }

    // ---- MOTOR COMMANDS (require motor ID) ----
    // Extract motor ID
    const char* motorId = doc["id"];

    if (motorId == nullptr) {
        sendErrorResponse("Missing 'id' field for motor command");
        return;
    }

    // Find the target motor
    MotorState* targetMotor = nullptr;
    if (strcmp(motorId, "motor-left") == 0) {
        targetMotor = &lftMotor;
    } else if (strcmp(motorId, "motor-right") == 0) {
        targetMotor = &rhtMotor;
    } else {
        sendErrorResponse("Unknown motor ID: " + String(motorId));
        return;
    }

    // Handle motor commands
    if (strcmp(command, "start") == 0) {
        // Check for duration option
        unsigned long duration = 0;

        if (doc.containsKey("options") && doc["options"].containsKey("duration")) {
            duration = doc["options"]["duration"].as<unsigned long>();
        }

        startMotor(targetMotor, duration);

        if (duration > 0) {
            sendSuccessResponse(String(motorId) + " started for " + String(duration) + "ms");
        } else {
            sendSuccessResponse(String(motorId) + " started (indefinite)");
        }

    } else if (strcmp(command, "stop") == 0) {
        stopMotor(targetMotor);
        sendSuccessResponse(String(motorId) + " stopped");
    } else {
        sendErrorResponse("Unknown command: " + String(command));
    }
}

// ============================================
// SERVO FUNCTIONS
// ============================================

void handleServoCommand(StaticJsonDocument<512>& doc) {
    // Check for angle option
    if (!doc.containsKey("options") || !doc["options"].containsKey("angle")) {
        sendErrorResponse("Missing 'angle' in options for turn command");
        return;
    }

    int targetAngle = doc["options"]["angle"].as<int>();

    // Validate angle range
    if (targetAngle < 0 || targetAngle > 180) {
        sendErrorResponse("Angle must be between 0 and 180 degrees");
        return;
    }

    // Check for optional speed parameter (degrees per step, with delay)
    int speed = 0;  // 0 means instant movement
    if (doc["options"].containsKey("speed")) {
        speed = doc["options"]["speed"].as<int>();
        if (speed < 1) speed = 1;
        if (speed > 100) speed = 100;
    }

    // Move servo
    if (speed > 0) {
        // Smooth movement
        moveServoSmooth(targetAngle, speed);
    } else {
        // Instant movement
        servoMotor.write(targetAngle);
        currentServoAngle = targetAngle;
    }

    // Send response
    StaticJsonDocument<128> response;
    response["success"] = true;
    response["message"] = "Servo moved";
    response["angle"] = currentServoAngle;

    serializeJson(response, Serial);
    Serial.println();

    Serial.print("[INFO] Servo moved to ");
    Serial.print(currentServoAngle);
    Serial.println(" degrees");
}

void moveServoSmooth(int targetAngle, int speed) {
    int delayTime = map(speed, 1, 100, 50, 5);  // Higher speed = shorter delay

    if (currentServoAngle < targetAngle) {
        for (int angle = currentServoAngle; angle <= targetAngle; angle++) {
            servoMotor.write(angle);
            delay(delayTime);
        }
    } else {
        for (int angle = currentServoAngle; angle >= targetAngle; angle--) {
            servoMotor.write(angle);
            delay(delayTime);
        }
    }

    currentServoAngle = targetAngle;
}

// ============================================
// ULTRASONIC SENSOR FUNCTIONS
// ============================================

float measureDistance() {
    // Clear the trigger pin
    digitalWrite(USONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Send 10 microsecond pulse to trigger
    digitalWrite(USONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(USONIC_TRIG_PIN, LOW);

    // Read the echo pin, returns pulse duration in microseconds
    long duration = pulseIn(USONIC_ECHO_PIN, HIGH, USONIC_TIMEOUT_MS);

    // Calculate distance in centimeters
    // Distance = (Time x Speed of Sound) / 2
    float distance = (duration * SOUND_SPEED_CM_PER_MS) / 2.0;

    // Check for out of range or no echo
    if (duration == 0 || distance > MAX_DISTANCE_CM || distance <= 0) {
        return -1;  // Invalid reading
    }

    return distance;
}

void handleReadDistanceCommand() {
    float distance = measureDistance();

    StaticJsonDocument<128> response;
    response["success"] = true;

    if (distance < 0) {
        response["distance_cm"] = nullptr;
        response["message"] = "No object detected or out of range";
    } else {
        response["distance_cm"] = distance;
        response["distance_inch"] = distance * 0.393701;
        response["message"] = "Distance measured successfully";
    }

    serializeJson(response, Serial);
    Serial.println();

    Serial.print("[INFO] Distance: ");
    if (distance < 0) {
        Serial.println("No reading");
    } else {
        Serial.print(distance);
        Serial.println(" cm");
    }
}

void handleScanCommand(StaticJsonDocument<512>& doc) {
    // Get scan parameters
    int count = 5;  // Default number of readings
    int interval = 500;  // Default interval in milliseconds

    if (doc.containsKey("options")) {
        if (doc["options"].containsKey("count")) {
            count = doc["options"]["count"].as<int>();
            if (count < 1) count = 1;
            if (count > 50) count = 50;  // Limit to prevent too long scans
        }
        if (doc["options"].containsKey("interval")) {
            interval = doc["options"]["interval"].as<int>();
            if (interval < 100) interval = 100;  // Minimum 100ms between readings
        }
    }

    Serial.print("[INFO] Starting scan: ");
    Serial.print(count);
    Serial.print(" readings, ");
    Serial.print(interval);
    Serial.println("ms interval");

    // Perform scan
    StaticJsonDocument<1024> response;
    response["success"] = true;
    response["command"] = "scan";

    JsonArray readings = response.createNestedArray("readings");

    for (int i = 0; i < count; i++) {
        float distance = measureDistance();
        
        JsonObject reading = readings.createNestedObject();
        reading["index"] = i;
        reading["timestamp_ms"] = millis();
        
        if (distance < 0) {
            reading["distance_cm"] = nullptr;
            reading["valid"] = false;
        } else {
            reading["distance_cm"] = distance;
            reading["valid"] = true;
        }
        
        // Wait before next reading (except for last one)
        if (i < count - 1) {
            delay(interval);
        }
    }

    serializeJson(response, Serial);
    Serial.println();

    Serial.println("[INFO] Scan complete");
}

// ============================================
// MOTOR FUNCTIONS
// ============================================

void startMotor(MotorState* motor, unsigned long duration) {
    digitalWrite(motor->pin, HIGH);
    motor->running = true;

    if (duration > 0) {
        motor->stopTime = millis() + duration;
    } else {
        motor->stopTime = 0;  // 0 means run indefinitely
    }

    Serial.print("[INFO] ");
    Serial.print(motor->id);
    Serial.print(" started on pin ");
    Serial.println(motor->pin);
}

void stopMotor(MotorState* motor) {
    digitalWrite(motor->pin, LOW);
    motor->running = false;
    motor->stopTime = 0;

    Serial.print("[INFO] ");
    Serial.print(motor->id);
    Serial.println(" stopped");
}

void stopAllMotors() {
    stopMotor(&rhtMotor);
    stopMotor(&lftMotor);
    Serial.println("[INFO] All motors stopped");
}

void checkMotorTimers() {
    unsigned long currentTime = millis();

    // Check right Motor
    if (rhtMotor.running && rhtMotor.stopTime > 0 && currentTime >= rhtMotor.stopTime) {
        stopMotor(&rhtMotor);
        sendTimerExpiredNotification(&rhtMotor);
    }

    // Check left Motor
    if (lftMotor.running && lftMotor.stopTime > 0 && currentTime >= lftMotor.stopTime) {
        stopMotor(&lftMotor);
        sendTimerExpiredNotification(&lftMotor);
    }
}

// ============================================
// RESPONSE FUNCTIONS
// ============================================

void sendSuccessResponse(String message) {
    StaticJsonDocument<128> response;
    response["success"] = true;
    response["message"] = message;

    serializeJson(response, Serial);
    Serial.println();
}

void sendErrorResponse(String message) {
    StaticJsonDocument<128> response;
    response["success"] = false;
    response["error"] = message;

    serializeJson(response, Serial);
    Serial.println();
}

void sendStatusResponse() {
    StaticJsonDocument<512> response;
    response["success"] = true;

    // Motors status
    JsonObject motors = response.createNestedObject("motors");

    JsonObject rhtMotorStatus = motors.createNestedObject("motor-right");
    rhtMotorStatus["running"] = rhtMotor.running;
    rhtMotorStatus["pin"] = rhtMotor.pin;
    if (rhtMotor.running && rhtMotor.stopTime > 0) {
        long remaining = rhtMotor.stopTime - millis();
        rhtMotorStatus["remaining_ms"] = remaining > 0 ? remaining : 0;
    }

    JsonObject lftMotorStatus = motors.createNestedObject("motor-left");
    lftMotorStatus["running"] = lftMotor.running;
    lftMotorStatus["pin"] = lftMotor.pin;
    if (lftMotor.running && lftMotor.stopTime > 0) {
        long remaining = lftMotor.stopTime - millis();
        lftMotorStatus["remaining_ms"] = remaining > 0 ? remaining : 0;
    }

    // Servo status
    JsonObject servoStatus = response.createNestedObject("servo");
    servoStatus["angle"] = currentServoAngle;
    servoStatus["pin"] = SERVO_SG90_PIN;

    // Ultrasonic sensor - take a reading
    float distance = measureDistance();
    JsonObject ultrasonicStatus = response.createNestedObject("ultrasonic");
    ultrasonicStatus["trig_pin"] = USONIC_TRIG_PIN;
    ultrasonicStatus["echo_pin"] = USONIC_ECHO_PIN;
    if (distance < 0) {
        ultrasonicStatus["distance_cm"] = nullptr;
    } else {
        ultrasonicStatus["distance_cm"] = distance;
    }

    serializeJson(response, Serial);
    Serial.println();
}

void sendTimerExpiredNotification(MotorState* motor) {
    StaticJsonDocument<128> notification;
    notification["event"] = "timer_expired";
    notification["motor"] = motor->id;

    serializeJson(notification, Serial);
    Serial.println();
}