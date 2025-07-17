/*
 * ESP32 PWM to ODrive Serial (ASCII) Converter
 * * Author: Gemini
 * * Date: July 12, 2025
 * * Version: 3.0 (With Throttle-Scaled Steering Mixer)
 * *
 * * Description:
 * This code reads throttle and steering PWM signals from an RC receiver and
 * converts them into scaled torque commands for a differential drive robot.
 * The steering input is scaled down as throttle increases to provide more
 * stable control at high speeds.
 */

#include <Arduino.h>


//================================================================================
// Configuration
//================================================================================

// --- PWM Input Pin Definitions ---
// <<< Connect your throttle channel to 13 and steering to 12
const int THROTTLE_PIN = 13; // For forward/reverse
const int STEERING_PIN = 12; // For left/right

// --- ODrive Serial Port Configuration ---
#define ODRIVE_SERIAL Serial2
const int ODRIVE_RX_PIN = 16;
const int ODRIVE_TX_PIN = 17;
const long ODRIVE_BAUD_RATE = 115200;

// --- Control & Timing Configuration ---
const unsigned int DEFAULT_PULSE_US = 1500;
const unsigned int MIN_PULSE_US = 1000;
const unsigned int MAX_PULSE_US = 2000;
const int PULSE_DEADZONE_US = 10; // Increased deadzone slightly for better stability

// <<< NEW: TUNING PARAMETERS
// Max torque in [Nm]. Adjust this to set the overall power of your vehicle.
const float MAX_TORQUE = 30.0; 
// Steering sensitivity scaling factor. 0.0 = full steering always. 1.0 = no steering at max throttle.
// A good starting point is 0.75
const float STEERING_SENSITIVITY = 0.5; 

// How often to send commands to ODrive (in milliseconds). 20ms = 50 Hz.
const unsigned long COMMAND_INTERVAL_MS = 20;
const unsigned long FAILSAFE_TIMEOUT_US = 100000; // 100 ms

//================================================================================
// Global Variables (Renamed for clarity)
//================================================================================

volatile unsigned long g_pulse_throttle_start_time = 0;
volatile unsigned long g_pulse_steering_start_time = 0;
volatile unsigned int g_pulse_throttle_width_us = DEFAULT_PULSE_US;
volatile unsigned int g_pulse_steering_width_us = DEFAULT_PULSE_US;
volatile unsigned long g_last_pulse_throttle_update = 0;
volatile unsigned long g_last_pulse_steering_update = 0;

unsigned long g_last_command_time = 0;

//================================================================================
// Interrupt Service Routines (ISRs) for PWM Input (Updated for new names)
//================================================================================

void IRAM_ATTR isr_throttle() {
  if (digitalRead(THROTTLE_PIN) == HIGH) {
    g_pulse_throttle_start_time = micros();
  } else {
    g_pulse_throttle_width_us = micros() - g_pulse_throttle_start_time;
    g_last_pulse_throttle_update = micros();
  }
}

void IRAM_ATTR isr_steering() {
  if (digitalRead(STEERING_PIN) == HIGH) {
    g_pulse_steering_start_time = micros();
  } else {
    g_pulse_steering_width_us = micros() - g_pulse_steering_start_time;
    g_last_pulse_steering_update = micros();
  }
}

//================================================================================
// Helper Function
//================================================================================
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//================================================================================
// Main Program: setup() and loop()
//================================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Differential Drive Mixer...");

  ODRIVE_SERIAL.begin(ODRIVE_BAUD_RATE, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);

  // Note: Assumes you've already set your ODrive axes to TORQUE_CONTROL mode
  // odrv0.axisN.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL

  pinMode(THROTTLE_PIN, INPUT_PULLUP);
  pinMode(STEERING_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), isr_throttle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEERING_PIN), isr_steering, CHANGE);
  
  Serial.println("Ready for RC input.");
}

void loop() {
  if (millis() - g_last_command_time >= COMMAND_INTERVAL_MS) {
    g_last_command_time = millis();

    // --- Get Pulse Data Safely ---
    noInterrupts();
    unsigned int pulse_throttle = g_pulse_throttle_width_us;
    unsigned int pulse_steering = g_pulse_steering_width_us;
    // (Failsafe logic would also be here)
    interrupts();
    
    // --- Sanity Check, Failsafe, and Deadzone (Apply to both inputs) ---
    // (This part of your code would be here and work on pulse_throttle and pulse_steering)
    if (abs((int)pulse_throttle - (int)DEFAULT_PULSE_US) <= PULSE_DEADZONE_US) pulse_throttle = DEFAULT_PULSE_US;
    if (abs((int)pulse_steering - (int)DEFAULT_PULSE_US) <= PULSE_DEADZONE_US) pulse_steering = DEFAULT_PULSE_US;


    // <<< --- NEW: NORMALIZE AND MIX INPUTS --- >>>

    // 1. Normalize both inputs to a -1.0 to 1.0 range
    float throttle_input = map_float(pulse_throttle, MIN_PULSE_US, MAX_PULSE_US, 1.0, -1.0);
    float steering_input = map_float(pulse_steering, MIN_PULSE_US, MAX_PULSE_US, -1.0, 1.0);
    throttle_input = constrain(throttle_input, -1.0, 1.0);
    steering_input = constrain(steering_input, -1.0, 1.0);

    // 2. Scale steering based on throttle
    // This formula reduces steering authority as you approach full throttle
    float steering_scale_factor = 1.0 - (STEERING_SENSITIVITY * abs(throttle_input));
    float scaled_steering = steering_input * steering_scale_factor;

    // 3. Mix throttle and scaled steering for left and right motors
    // Assumes motor 0 is RIGHT and motor 1 is LEFT
    float right_motor_norm = throttle_input - scaled_steering;
    float left_motor_norm = throttle_input + scaled_steering;

    // 4. Clamp values to ensure they are within the -1.0 to 1.0 range
    right_motor_norm = constrain(right_motor_norm, -1.0, 1.0);
    left_motor_norm = constrain(left_motor_norm, -1.0, 1.0);
    
    // 5. Map the normalized values to the final torque commands
    float motor_right_torque = right_motor_norm * MAX_TORQUE;
    float motor_left_torque = left_motor_norm * MAX_TORQUE;
    
    // --- Format and Send Serial Commands to ODrive ---
    // Assuming axis 0 is the RIGHT motor and axis 1 is the LEFT motor
    String command_right = "c 0 " + String(motor_right_torque, 2) + "\n";
    String command_left = "c 1 " + String(motor_left_torque, 2) + "\n";

    ODRIVE_SERIAL.print(command_right);
    ODRIVE_SERIAL.print(command_left);
    
    // --- Optional: Print debug info to USB serial ---
    Serial.print("T: "); Serial.print(throttle_input, 2);
    Serial.print(" | S: "); Serial.print(steering_input, 2);
    Serial.print(" | Scaled S: "); Serial.print(scaled_steering, 2);
    Serial.print(" => R: "); Serial.print(motor_right_torque, 2);
    Serial.print(" | L: "); Serial.println(motor_left_torque, 2);
  }
}