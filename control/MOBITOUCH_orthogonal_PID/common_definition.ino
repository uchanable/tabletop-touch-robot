#include "HX711.h"
#include <Dynamixel2Arduino.h>

// For mobile robot control
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

// ==========================================
// HX711 common definitions
// ==========================================
const int LOADCELL_DOUT_PIN = 8;
const int LOADCELL_SCK_PIN = 9;
HX711 scale;

// ==========================================
// Dynamixel common definitions  
// ==========================================
#define DXL_SERIAL   Serial1
// DEBUG_SERIAL is defined individually for each mode
const int DXL_DIR_PIN = 28;
const uint8_t MOTOR_ID = 1; // Force control motor
const uint8_t INTEG_LEFT_MOTOR_ID = 2;  // For integrated control (left)
const uint8_t INTEG_RIGHT_MOTOR_ID = 3; // For integrated control (right)
const float DXL_PROTOCOL_VERSION = 2.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// Mobile robot control object
QwiicOTOS myOtos;

// ==========================================
// Common utility functions
// ==========================================

void printSeparator() {
  Serial.println("========================================");
}

void printHeader(String title) {
  printSeparator();
  Serial.print("=== ");
  Serial.print(title);
  Serial.println(" ===");
  printSeparator();
}

void printStatus(String message) {
  Serial.print("Status: ");
  Serial.println(message);
}

void printError(String message) {
  Serial.print("Error: ");
  Serial.println(message);
}

void printSuccess(String message) {
  Serial.print("Success: ");
  Serial.println(message);
}

float constrainFloat(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

float absFloat(float value) {
  return value < 0 ? -value : value;
}

class IntegPIDController {
private:
  float kp, ki, kd;
  float setpoint;
  float previous_error;
  float integral;
  unsigned long last_time;
  float integral_max;
  float integral_min;
  int startup_cycles;
  
public:
  IntegPIDController(float _kp = 0.0, float _ki = 0.0, float _kd = 0.0, float _setpoint = 0.0) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
    setpoint = _setpoint;
    previous_error = 0.0;
    integral = 0.0;
    last_time = millis();
    integral_max = 800.0;
    integral_min = -800.0;
    startup_cycles = 0;
  }
  
  float update(float current_value) {
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0;
    
    if (dt <= 0.0) dt = 0.001;
    float error = setpoint - current_value;
    float proportional = kp * error;
    
    integral += error * dt;
    if (integral > integral_max) integral = integral_max;
    else if (integral < integral_min) integral = integral_min;
    float integral_term = ki * integral;
    
    float derivative;
    if (startup_cycles > 0) {
      derivative = 0;
      startup_cycles--;
    } else {
      derivative = kd * (error - previous_error) / dt;
    }
    
    float output = proportional + integral_term + derivative;
    previous_error = error;
    last_time = current_time;
    return output;
  }
  
  void reset(float current_error = 0.0) {
    previous_error = current_error;
    integral = 0.0;
    last_time = millis();
    startup_cycles = 5;
  }
  
  void setSetpoint(float new_setpoint) {
    setpoint = new_setpoint;
  }
};


// ==========================================
// Common motor control functions (ID=1, for force control motor)
// ==========================================

// Get current angle [degrees] (XL-320: 0-1023 -> -150 to 150 degrees)
float getCurrentAngle() {
  int current_position = dxl.getPresentPosition(MOTOR_ID);
  return (current_position - 512) * 300.0 / 1024.0;
}

// Move to specified angle [degrees]
void moveToAngle(float angle) {
  int position = (int)(512 + (angle * 1024.0 / 300.0));
  position = constrain(position, 0, 1023);
  dxl.setGoalPosition(MOTOR_ID, position);
}


// ==========================================
// Common initialization functions
// ==========================================

// Simple initialization of Dynamixel (ID=1)
bool initializeDynamixel_ID1() {
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  delay(100);
  
  if (!dxl.ping(MOTOR_ID)) {
    printError("Motor (ID=1) connection failed");
    return false;
  }
  printSuccess("Motor (ID=1) connection confirmed");
  
  dxl.torqueOff(MOTOR_ID);
  dxl.setOperatingMode(MOTOR_ID, OP_POSITION); // Position control mode
  uint16_t cw_limit = 0;
  uint16_t ccw_limit = 1023;
  dxl.write(MOTOR_ID, 6, (uint8_t*)&cw_limit, 2);
  dxl.write(MOTOR_ID, 8, (uint8_t*)&ccw_limit, 2);
  dxl.torqueOn(MOTOR_ID);
  return true;
}
