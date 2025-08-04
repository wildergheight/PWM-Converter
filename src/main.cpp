/*
 * ESP32 PWM to ODrive Serial (ASCII) Converter
 * * Author: Gemini
 * * Date: August 1, 2025
 * * Version: 6.1 (With Torque Rate Limiter)
 * *
 * * Description:
 * This version adds a torque rate limiter (slew rate) to prevent sudden,
 * jerky movements caused by errant RC signals or processing glitches. It smooths
 * the motor commands for more reliable operation.
 */

#include <Arduino.h>
#include <math.h> // Required for powf and copysignf functions

//================================================================================
// Configuration
//================================================================================

// --- PWM Input Pin Definitions ---
const int THROTTLE_PIN = 13;
const int STEERING_PIN = 12;
const int BRAKE_PIN = 14;

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
const unsigned int BRAKE_ON_THRESHOLD_US = 1500;

// --- Tuning Parameters ---
const float MAX_TORQUE = 30.0;
const float STEERING_SENSITIVITY = 0.5;
const float THROTTLE_EXPO = 2.0;

// <<< NEW: TORQUE RATE LIMITER >>>
// Max change in torque allowed per command cycle (e.g., per 20ms).
// Lower values = smoother/slower acceleration. Higher = more responsive.
// This prevents motor "jerks" from bad signals. A good starting point is 1.0.
const float MAX_TORQUE_CHANGE_PER_CYCLE = 1.0;


// --- Failsafe and Timing ---
const unsigned long COMMAND_INTERVAL_MS = 20;
const unsigned long FAILSAFE_TIMEOUT_US = 100000;

//================================================================================
// Global Variables
//================================================================================

enum ControlState { STATE_TORQUE_CONTROL, STATE_BRAKING };
ControlState g_control_state = STATE_TORQUE_CONTROL;

volatile unsigned long g_pulse_throttle_start_time, g_pulse_steering_start_time, g_pulse_brake_start_time;
volatile unsigned int g_pulse_throttle_width_us = DEFAULT_PULSE_US;
volatile unsigned int g_pulse_steering_width_us = DEFAULT_PULSE_US;
volatile unsigned int g_pulse_brake_width_us = MIN_PULSE_US;
volatile unsigned long g_last_pulse_throttle_update, g_last_pulse_steering_update, g_last_pulse_brake_update;

unsigned long g_last_command_time = 0;

// <<< NEW: State for Rate Limiter >>>
float g_last_sent_right_torque = 0.0;
float g_last_sent_left_torque = 0.0;

//================================================================================
// Interrupt Service Routines (ISRs) for PWM Input
//================================================================================
void IRAM_ATTR isr_throttle() { if (digitalRead(THROTTLE_PIN)) g_pulse_throttle_start_time = micros(); else { g_pulse_throttle_width_us = micros() - g_pulse_throttle_start_time; g_last_pulse_throttle_update = micros(); } }
void IRAM_ATTR isr_steering() { if (digitalRead(STEERING_PIN)) g_pulse_steering_start_time = micros(); else { g_pulse_steering_width_us = micros() - g_pulse_steering_start_time; g_last_pulse_steering_update = micros(); } }
void IRAM_ATTR isr_brake() { if (digitalRead(BRAKE_PIN)) g_pulse_brake_start_time = micros(); else { g_pulse_brake_width_us = micros() - g_pulse_brake_start_time; g_last_pulse_brake_update = micros(); } }

//================================================================================
// Helper Function
//================================================================================
float map_float(float x, float in_min, float in_max, float out_min, float out_max) { return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }

//================================================================================
// Main Program: setup() and loop()
//================================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Differential Drive Mixer v6.1 (Rate Limiter)...");
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
    return;
  }
  g_last_command_time = millis();

  // --- STATE MACHINE LOGIC ---
  noInterrupts();
  unsigned long last_throttle_us = g_last_pulse_throttle_update;
  unsigned int pulse_brake = g_pulse_brake_width_us;
  interrupts();

  unsigned long now_us = micros();
  bool failsafe_triggered = (now_us - last_throttle_us > FAILSAFE_TIMEOUT_US);

  bool brake_switch_on = (pulse_brake > BRAKE_ON_THRESHOLD_US);
  ControlState desired_state = (failsafe_triggered || brake_switch_on) ? STATE_BRAKING : STATE_TORQUE_CONTROL;

  if (desired_state != g_control_state) {
    if (desired_state == STATE_BRAKING) {
      Serial.println(failsafe_triggered ? "FAILSAFE: Engaging brake." : "SWITCH: Engaging brake.");
      ODRIVE_SERIAL.println("w 0.controller.config.control_mode 2");
      ODRIVE_SERIAL.println("w 1.controller.config.control_mode 2");
      // Reset last sent torque when braking is engaged
      g_last_sent_right_torque = 0.0;
      g_last_sent_left_torque = 0.0;
    } else {
      Serial.println("RELEASED: Switching to Torque Control.");
      ODRIVE_SERIAL.println("w 0.controller.config.control_mode 1");
      ODRIVE_SERIAL.println("w 1.controller.config.control_mode 1");
    }
    g_control_state = desired_state;
    delay(5);
  }

  // --- ACTION BASED ON CURRENT STATE ---
  if (g_control_state == STATE_BRAKING) {
    ODRIVE_SERIAL.println("v 0 0");
    ODRIVE_SERIAL.println("v 1 0");
  } else { // STATE_TORQUE_CONTROL
    noInterrupts();
    unsigned int pulse_throttle = g_pulse_throttle_width_us;
    unsigned int pulse_steering = g_pulse_steering_width_us;
    interrupts();

    if (abs((int)pulse_throttle - (int)DEFAULT_PULSE_US) <= PULSE_DEADZONE_US) pulse_throttle = DEFAULT_PULSE_US;
    if (abs((int)pulse_steering - (int)DEFAULT_PULSE_US) <= PULSE_DEADZONE_US) pulse_steering = DEFAULT_PULSE_US;

    float throttle_input = map_float(pulse_throttle, MIN_PULSE_US, MAX_PULSE_US, 1.0, -1.0);
    float steering_input = map_float(pulse_steering, MIN_PULSE_US, MAX_PULSE_US, -1.0, 1.0);
    throttle_input = constrain(throttle_input, -1.0, 1.0);
    steering_input = constrain(steering_input, -1.0, 1.0);

    float steering_scale_factor = 1.0 - (STEERING_SENSITIVITY * abs(throttle_input));
    float scaled_steering = steering_input * steering_scale_factor;

    float right_motor_norm = constrain(throttle_input - scaled_steering, -1.0, 1.0);
    float left_motor_norm = constrain(throttle_input + scaled_steering, -1.0, 1.0);

    float curved_right_norm = copysignf(powf(fabsf(right_motor_norm), THROTTLE_EXPO), right_motor_norm);
    float curved_left_norm = copysignf(powf(fabsf(left_motor_norm), THROTTLE_EXPO), left_motor_norm);

    // Calculate the raw desired torque
    float desired_right_torque = curved_right_norm * MAX_TORQUE;
    float desired_left_torque = curved_left_norm * MAX_TORQUE;

    // <<< NEW: Apply Rate Limiter >>>
    // Calculate the change from the last sent command
    float right_torque_change = desired_right_torque - g_last_sent_right_torque;
    float left_torque_change = desired_left_torque - g_last_sent_left_torque;

    // Clamp the change to the maximum allowed step
    right_torque_change = constrain(right_torque_change, -MAX_TORQUE_CHANGE_PER_CYCLE, MAX_TORQUE_CHANGE_PER_CYCLE);
    left_torque_change = constrain(left_torque_change, -MAX_TORQUE_CHANGE_PER_CYCLE, MAX_TORQUE_CHANGE_PER_CYCLE);

    // The final torque is the last sent value plus the allowed change
    float final_right_torque = g_last_sent_right_torque + right_torque_change;
    float final_left_torque = g_last_sent_left_torque + left_torque_change;

    // --- Format and Send Serial Commands to ODrive ---
    String command_right = "c 0 " + String(final_right_torque, 2) + "\n";
    String command_left = "c 1 " + String(final_left_torque, 2) + "\n";

    ODRIVE_SERIAL.print(command_right);
    ODRIVE_SERIAL.print(command_left);

    // Update the state for the next cycle
    g_last_sent_right_torque = final_right_torque;
    g_last_sent_left_torque = final_left_torque;
  }
}
