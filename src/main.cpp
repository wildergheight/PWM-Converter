/*
 * ESP32 PWM to ODrive Serial (ASCII) Converter
 * * Author: Gemini
 * * Date: July 16, 2025
 * * Version: 5.1 (Corrected State Machine Logic)
 * *
 * * Description:
 * This code reads throttle, steering, and a brake switch PWM signal from an
 * RC receiver. It converts throttle/steering to torque commands for a
 * differential drive robot. The brake channel activates a velocity-hold mode.
 * A failsafe is included to brake the robot if RC signal is lost.
 * This version corrects a bug that prevented switching back to torque control.
 */

#include <Arduino.h>


//================================================================================
// Configuration
//================================================================================

// --- PWM Input Pin Definitions ---
const int THROTTLE_PIN = 13; // For forward/reverse
const int STEERING_PIN = 12; // For left/right
const int BRAKE_PIN = 14;    // For RC channel brake switch (e.g., CH5)

// --- ODrive Serial Port Configuration ---
#define ODRIVE_SERIAL Serial2
const int ODRIVE_RX_PIN = 16;
const int ODRIVE_TX_PIN = 17;
const long ODRIVE_BAUD_RATE = 115200;

// --- Control & Timing Configuration ---
const unsigned int DEFAULT_PULSE_US = 1500;
const unsigned int MIN_PULSE_US = 1000;
const unsigned int MAX_PULSE_US = 2000;
const int PULSE_DEADZONE_US = 10;
const unsigned int BRAKE_ON_THRESHOLD_US = 1500; // Brake is ON if pulse > 1500us

// --- Tuning Parameters ---
const float MAX_TORQUE = 30.0;
const float STEERING_SENSITIVITY = 0.5;

// --- Failsafe and Timing ---
const unsigned long COMMAND_INTERVAL_MS = 20;    // 50 Hz command rate
const unsigned long FAILSAFE_TIMEOUT_US = 100000; // 100 ms

//================================================================================
// Global Variables
//================================================================================

// --- State Machine for Control ---
enum ControlState { STATE_TORQUE_CONTROL, STATE_BRAKING };
ControlState g_control_state = STATE_TORQUE_CONTROL;

// --- PWM Input Variables ---
volatile unsigned long g_pulse_throttle_start_time = 0;
volatile unsigned long g_pulse_steering_start_time = 0;
volatile unsigned long g_pulse_brake_start_time = 0;

volatile unsigned int g_pulse_throttle_width_us = DEFAULT_PULSE_US;
volatile unsigned int g_pulse_steering_width_us = DEFAULT_PULSE_US;
volatile unsigned int g_pulse_brake_width_us = MIN_PULSE_US;

volatile unsigned long g_last_pulse_throttle_update = 0;
volatile unsigned long g_last_pulse_steering_update = 0;
volatile unsigned long g_last_pulse_brake_update = 0;

unsigned long g_last_command_time = 0;

//================================================================================
// Interrupt Service Routines (ISRs) for PWM Input
//================================================================================

void IRAM_ATTR isr_throttle() {
  if (digitalRead(THROTTLE_PIN) == HIGH) g_pulse_throttle_start_time = micros();
  else {
    g_pulse_throttle_width_us = micros() - g_pulse_throttle_start_time;
    g_last_pulse_throttle_update = micros();
  }
}

void IRAM_ATTR isr_steering() {
  if (digitalRead(STEERING_PIN) == HIGH) g_pulse_steering_start_time = micros();
  else {
    g_pulse_steering_width_us = micros() - g_pulse_steering_start_time;
    g_last_pulse_steering_update = micros();
  }
}

void IRAM_ATTR isr_brake() {
  if (digitalRead(BRAKE_PIN) == HIGH) g_pulse_brake_start_time = micros();
  else {
    g_pulse_brake_width_us = micros() - g_pulse_brake_start_time;
    g_last_pulse_brake_update = micros();
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
  Serial.println("Starting Differential Drive Mixer v5.1 (Corrected Logic)...");

  ODRIVE_SERIAL.begin(ODRIVE_BAUD_RATE, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);

  pinMode(THROTTLE_PIN, INPUT_PULLUP);
  pinMode(STEERING_PIN, INPUT_PULLUP);
  pinMode(BRAKE_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), isr_throttle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEERING_PIN), isr_steering, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BRAKE_PIN), isr_brake, CHANGE);

  Serial.println("Ready for RC input.");
}

void loop() {
  if (millis() - g_last_command_time < COMMAND_INTERVAL_MS) {
    return; // If it's not time yet, exit
  }
  g_last_command_time = millis();

  // <<< --- REFACTORED STATE MACHINE LOGIC --- >>>

  // 1. Read all inputs and check for failsafe
  noInterrupts();
  unsigned long last_throttle_us = g_last_pulse_throttle_update;
  unsigned long last_steering_us = g_last_pulse_steering_update;
  unsigned long last_brake_us = g_last_pulse_brake_update;
  unsigned int pulse_brake = g_pulse_brake_width_us;
  interrupts();

  unsigned long now_us = micros();
  bool failsafe_triggered = (now_us - last_throttle_us > FAILSAFE_TIMEOUT_US) ||
                            (now_us - last_steering_us > FAILSAFE_TIMEOUT_US) ||
                            (now_us - last_brake_us > FAILSAFE_TIMEOUT_US);

  bool brake_switch_on = (pulse_brake > BRAKE_ON_THRESHOLD_US);

  // 2. Determine the single desired state based on all conditions
  ControlState desired_state = (failsafe_triggered || brake_switch_on) ? STATE_BRAKING : STATE_TORQUE_CONTROL;

  // 3. If the desired state is different from the current state, send mode change commands
  if (desired_state != g_control_state) {
    if (desired_state == STATE_BRAKING) {
      Serial.println(failsafe_triggered ? "FAILSAFE: Engaging brake." : "SWITCH: Engaging brake.");
      // Switch both axes to velocity control for braking
      ODRIVE_SERIAL.println("w 0.controller.config.control_mode 2");
      ODRIVE_SERIAL.println("w 1.controller.config.control_mode 2");
    } else { // Switching back to torque control
      Serial.println("RELEASED: Switching to Torque Control.");
      // Switch both axes back to torque control
      ODRIVE_SERIAL.println("w 0.controller.config.control_mode 1");
      ODRIVE_SERIAL.println("w 1.controller.config.control_mode 1");
    }
    g_control_state = desired_state; // Update the global state
    delay(5); // Give ODrive a moment to process the mode change
  }

  // 4. Perform action based on the current (and now correct) state
  if (g_control_state == STATE_BRAKING) {
    // Send zero velocity commands to both motors
    ODRIVE_SERIAL.println("v 0 0");
    ODRIVE_SERIAL.println("v 1 0");
  } else { // STATE_TORQUE_CONTROL
    // Get latest pulse data safely
    noInterrupts();
    unsigned int pulse_throttle = g_pulse_throttle_width_us;
    unsigned int pulse_steering = g_pulse_steering_width_us;
    interrupts();

    // Apply deadzone
    if (abs((int)pulse_throttle - (int)DEFAULT_PULSE_US) <= PULSE_DEADZONE_US) pulse_throttle = DEFAULT_PULSE_US;
    if (abs((int)pulse_steering - (int)DEFAULT_PULSE_US) <= PULSE_DEADZONE_US) pulse_steering = DEFAULT_PULSE_US;

    // --- Normalize and Mix Inputs ---
    float throttle_input = map_float(pulse_throttle, MIN_PULSE_US, MAX_PULSE_US, 1.0, -1.0);
    float steering_input = map_float(pulse_steering, MIN_PULSE_US, MAX_PULSE_US, -1.0, 1.0);
    throttle_input = constrain(throttle_input, -1.0, 1.0);
    steering_input = constrain(steering_input, -1.0, 1.0);

    float steering_scale_factor = 1.0 - (STEERING_SENSITIVITY * abs(throttle_input));
    float scaled_steering = steering_input * steering_scale_factor;

    float right_motor_norm = throttle_input - scaled_steering;
    float left_motor_norm = throttle_input + scaled_steering;
    right_motor_norm = constrain(right_motor_norm, -1.0, 1.0);
    left_motor_norm = constrain(left_motor_norm, -1.0, 1.0);

    float motor_right_torque = right_motor_norm * MAX_TORQUE;
    float motor_left_torque = left_motor_norm * MAX_TORQUE;

    String command_right = "c 0 " + String(motor_right_torque, 2) + "\n";
    String command_left = "c 1 " + String(motor_left_torque, 2) + "\n";

    ODRIVE_SERIAL.print(command_right);
    ODRIVE_SERIAL.print(command_left);
  }
}