#include <ArduinoJson.h>

// Motor pin definitions (using ELEGOO ESP32 labels)
const int MOTOR_A_PIN = D2;
const int MOTOR_B_PIN = D4;

// Motor state tracking
struct MotorState {
    bool running;
    unsigned long stopTime;
    int pin;
    String id;
};

MotorState motorA = {false, 0, MOTOR_A_PIN, "motor-a"};
MotorState motorB = {false, 0, MOTOR_B_PIN, "motor-b"};

void setup() {
    Serial.begin(115200);
    
    // Configure motor pins as outputs
    pinMode(MOTOR_A_PIN, OUTPUT);
    pinMode(MOTOR_B_PIN, OUTPUT);
    
    // Ensure motors are OFF at startup
    digitalWrite(MOTOR_A_PIN, LOW);
    digitalWrite(MOTOR_B_PIN, LOW);
    
    Serial.println();
    Serial.println("=================================");
    Serial.println("  Motor Control System Ready");
    Serial.println("=================================");
    Serial.println();
    Serial.println("Send JSON commands to control motors:");
    Serial.println();
    Serial.println("Start motor for duration:");
    Serial.println("  {\"id\":\"motor-a\",\"command\":\"start\",\"options\":{\"duration\":1200}}");
    Serial.println();
    Serial.println("Start motor indefinitely:");
    Serial.println("  {\"id\":\"motor-b\",\"command\":\"start\"}");
    Serial.println();
    Serial.println("Stop motor:");
    Serial.println("  {\"id\":\"motor-a\",\"command\":\"stop\"}");
    Serial.println();
    Serial.println("Stop all motors:");
    Serial.println("  {\"command\":\"stop-all\"}");
    Serial.println();
    Serial.println("Query status:");
    Serial.println("  {\"command\":\"status\"}");
    Serial.println();
}

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
}

void processCommand(String jsonString) {
    // Parse JSON
    StaticJsonDocument<256> doc;
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
    
    // Handle global commands
    if (strcmp(command, "stop-all") == 0) {
        stopAllMotors();
        sendSuccessResponse("All motors stopped");
        return;
    }
    
    if (strcmp(command, "status") == 0) {
        sendStatusResponse();
        return;
    }
    
    // For motor-specific commands, extract motor ID
    const char* motorId = doc["id"];
    
    if (motorId == nullptr) {
        sendErrorResponse("Missing 'id' field for motor command");
        return;
    }
    
    // Find the target motor
    MotorState* targetMotor = nullptr;
    
    if (strcmp(motorId, "motor-a") == 0) {
        targetMotor = &motorA;
    } else if (strcmp(motorId, "motor-b") == 0) {
        targetMotor = &motorB;
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
    stopMotor(&motorA);
    stopMotor(&motorB);
    Serial.println("[INFO] All motors stopped");
}

void checkMotorTimers() {
    unsigned long currentTime = millis();
    
    // Check Motor A
    if (motorA.running && motorA.stopTime > 0 && currentTime >= motorA.stopTime) {
        stopMotor(&motorA);
        sendTimerExpiredNotification(&motorA);
    }
    
    // Check Motor B
    if (motorB.running && motorB.stopTime > 0 && currentTime >= motorB.stopTime) {
        stopMotor(&motorB);
        sendTimerExpiredNotification(&motorB);
    }
}

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
    StaticJsonDocument<256> response;
    response["success"] = true;
    
    JsonObject motors = response.createNestedObject("motors");
    
    JsonObject motorAStatus = motors.createNestedObject("motor-a");
    motorAStatus["running"] = motorA.running;
    motorAStatus["pin"] = motorA.pin;
    if (motorA.running && motorA.stopTime > 0) {
        long remaining = motorA.stopTime - millis();
        motorAStatus["remaining_ms"] = remaining > 0 ? remaining : 0;
    }
    
    JsonObject motorBStatus = motors.createNestedObject("motor-b");
    motorBStatus["running"] = motorB.running;
    motorBStatus["pin"] = motorB.pin;
    if (motorB.running && motorB.stopTime > 0) {
        long remaining = motorB.stopTime - millis();
        motorBStatus["remaining_ms"] = remaining > 0 ? remaining : 0;
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