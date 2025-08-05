/*
 * ESP32 PWM/Serial to ODrive Serial (ASCII) Converter
 * * Author: Gemini
 * * Date: August 5, 2025
 * * Version: 7.1 (ROS2 Control with Advanced Features)
 * *
 * * Description:
 * This code acts as a bridge between an RC receiver, a ROS2-enabled SBC,
 * and an ODrive. It incorporates an exponential throttle curve and a torque
 * rate limiter for smooth operation.
 *
 * Control Priority:
 * 1. RC Brake/Failsafe (Highest): Brakes the motors.
 * 2. ROS2 Serial Command: Drives the motors based on commands from USB.
 * 3. RC Control (Lowest): Drives motors if no ROS2 command is received.
 */

#include <Arduino.h>
#include <math.h>

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

// --- ROS2 Serial Port (Main USB) ---
#define ROS_SERIAL Serial
const long ROS_BAUD_RATE = 115200;

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
const float MAX_TORQUE_CHANGE_PER_CYCLE = 1.0;

// --- Failsafe and Timing ---
const unsigned long COMMAND_INTERVAL_MS = 20;
const unsigned long RC_FAILSAFE_TIMEOUT_US = 100000; // 100 ms
const unsigned long ROS_FAILSAFE_TIMEOUT_MS = 500;  // Fallback to RC after 500ms

//================================================================================
// Global Variables
//================================================================================

// --- State Machine for Control ---
enum ControlState { STATE_TORQUE_RC, STATE_TORQUE_ROS, STATE_BRAKING };
ControlState g_control_state = STATE_TORQUE_RC;

// --- PWM Input Variables ---
volatile unsigned long g_pulse_throttle_start_time, g_pulse_steering_start_time, g_pulse_brake_start_time;
volatile unsigned int g_pulse_throttle_width_us = DEFAULT_PULSE_US;
volatile unsigned int g_pulse_steering_width_us = DEFAULT_PULSE_US;
volatile unsigned int g_pulse_brake_width_us = MIN_PULSE_US;
volatile unsigned long g_last_pulse_throttle_update, g_last_pulse_steering_update, g_last_pulse_brake_update;

// --- ROS2 Command Variables ---
String g_ros_serial_buffer = "";
float g_ros_right_norm = 0.0;
float g_ros_left_norm = 0.0;
unsigned long g_last_ros_command_time = 0;

// --- Rate Limiter State ---
float g_last_sent_right_torque = 0.0;
float g_last_sent_left_torque = 0.0;

unsigned long g_last_command_time = 0;

//================================================================================
// Interrupt Service Routines (ISRs) for PWM Input
//================================================================================
void IRAM_ATTR isr_throttle() { if (digitalRead(THROTTLE_PIN)) g_pulse_throttle_start_time = micros(); else { g_pulse_throttle_width_us = micros() - g_pulse_throttle_start_time; g_last_pulse_throttle_update = micros(); } }
void IRAM_ATTR isr_steering() { if (digitalRead(STEERING_PIN)) g_pulse_steering_start_time = micros(); else { g_pulse_steering_width_us = micros() - g_pulse_steering_start_time; g_last_pulse_steering_update = micros(); } }
void IRAM_ATTR isr_brake() { if (digitalRead(BRAKE_PIN)) g_pulse_brake_start_time = micros(); else { g_pulse_brake_width_us = micros() - g_pulse_brake_start_time; g_last_pulse_brake_update = micros(); } }

//================================================================================
// ROS Serial Parsing
//================================================================================
void parseRosCommand(String cmd) {
    cmd.trim();
    int r_idx = cmd.indexOf('R');
    int l_idx = cmd.indexOf('L');
    int comma_idx = cmd.indexOf(',');

    if (r_idx != -1 && l_idx != -1 && comma_idx != -1) {
        String r_val_str = cmd.substring(r_idx + 2, comma_idx);
        String l_val_str = cmd.substring(l_idx + 2);
        
        g_ros_right_norm = constrain(r_val_str.toFloat(), -1.0, 1.0);
        g_ros_left_norm = constrain(l_val_str.toFloat(), -1.0, 1.0);
        g_last_ros_command_time = millis();
    }
}

void checkRosSerial() {
    while (ROS_SERIAL.available()) {
        char c = ROS_SERIAL.read();
        if (c == '\n') {
            parseRosCommand(g_ros_serial_buffer);
            g_ros_serial_buffer = "";
        } else {
            g_ros_serial_buffer += c;
        }
    }
}

//================================================================================
// Main Program
//================================================================================
void setup() {
    ROS_SERIAL.begin(ROS_BAUD_RATE);
    ODRIVE_SERIAL.begin(ODRIVE_BAUD_RATE, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);
    
    pinMode(THROTTLE_PIN, INPUT_PULLUP);
    pinMode(STEERING_PIN, INPUT_PULLUP);
    pinMode(BRAKE_PIN, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), isr_throttle, CHANGE);
    attachInterrupt(digitalPinToInterrupt(STEERING_PIN), isr_steering, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BRAKE_PIN), isr_brake, CHANGE);
    
    ROS_SERIAL.println("ESP32 ODrive Bridge v7.1 Ready. Listening for RC and ROS commands.");
}

void loop() {
    checkRosSerial();

    if (millis() - g_last_command_time < COMMAND_INTERVAL_MS) return;
    g_last_command_time = millis();

    // --- DETERMINE CONTROL STATE ---
    ControlState desired_state;
    
    noInterrupts();
    unsigned long last_rc_update = g_last_pulse_throttle_update;
    unsigned int brake_pulse = g_pulse_brake_width_us;
    interrupts();
    
    bool rc_failsafe = (micros() - last_rc_update > RC_FAILSAFE_TIMEOUT_US);
    bool brake_switch_on = (brake_pulse > BRAKE_ON_THRESHOLD_US);

    if (rc_failsafe || brake_switch_on) {
        desired_state = STATE_BRAKING;
    } else if (millis() - g_last_ros_command_time < ROS_FAILSAFE_TIMEOUT_MS) {
        desired_state = STATE_TORQUE_ROS;
    } else {
        desired_state = STATE_TORQUE_RC;
    }

    // --- HANDLE STATE TRANSITIONS ---
    if (desired_state != g_control_state) {
        const char* reason = "";
        if (desired_state == STATE_BRAKING) {
            reason = rc_failsafe ? "RC FAILSAFE" : "RC BRAKE";
            ROS_SERIAL.printf("STATE CHANGE: -> BRAKING (%s)\n", reason);
            ODRIVE_SERIAL.println("w 0.controller.config.control_mode 2"); // Velocity Control
            ODRIVE_SERIAL.println("w 1.controller.config.control_mode 2");
            g_last_sent_right_torque = 0.0; // Reset rate limiter
            g_last_sent_left_torque = 0.0;
        } else { // Switching to a torque control mode
            if (g_control_state == STATE_BRAKING) { // If coming out of brake
                ODRIVE_SERIAL.println("w 0.controller.config.control_mode 1"); // Torque Control
                ODRIVE_SERIAL.println("w 1.controller.config.control_mode 1");
            }
            reason = (desired_state == STATE_TORQUE_ROS) ? "ROS" : "RC";
            ROS_SERIAL.printf("STATE CHANGE: -> TORQUE (%s)\n", reason);
        }
        g_control_state = desired_state;
        delay(5);
    }

    // --- EXECUTE ACTION BASED ON STATE ---
    if (g_control_state == STATE_BRAKING) {
        ODRIVE_SERIAL.println("v 0 0");
        ODRIVE_SERIAL.println("v 1 0");
        return;
    }

    // --- Get normalized commands based on the current state ---
    float right_norm = 0.0, left_norm = 0.0;
    switch(g_control_state) {
        case STATE_TORQUE_ROS:
            right_norm = g_ros_right_norm;
            left_norm = g_ros_left_norm;
            break;

        case STATE_TORQUE_RC:
            noInterrupts();
            unsigned int pulse_throttle = g_pulse_throttle_width_us;
            unsigned int pulse_steering = g_pulse_steering_width_us;
            interrupts();

            if (abs((int)pulse_throttle - (int)DEFAULT_PULSE_US) <= PULSE_DEADZONE_US) pulse_throttle = DEFAULT_PULSE_US;
            if (abs((int)pulse_steering - (int)DEFAULT_PULSE_US) <= PULSE_DEADZONE_US) pulse_steering = DEFAULT_PULSE_US;
            
            float throttle_input = map_float(pulse_throttle, MIN_PULSE_US, MAX_PULSE_US, 1.0, -1.0);
            float steering_input = map_float(pulse_steering, MIN_PULSE_US, MAX_PULSE_US, -1.0, 1.0);
            
            float steering_scale_factor = 1.0 - (STEERING_SENSITIVITY * abs(throttle_input));
            float scaled_steering = steering_input * steering_scale_factor;

            right_norm = constrain(throttle_input - scaled_steering, -1.0, 1.0);
            left_norm = constrain(throttle_input + scaled_steering, -1.0, 1.0);
            break;
    }

    // --- APPLY UNIFIED PROCESSING (EXPO AND RATE LIMIT) ---
    float curved_right = copysignf(powf(fabsf(right_norm), THROTTLE_EXPO), right_norm);
    float curved_left = copysignf(powf(fabsf(left_norm), THROTTLE_EXPO), left_norm);

    float desired_right_torque = curved_right * MAX_TORQUE;
    float desired_left_torque = curved_left * MAX_TORQUE;

    float right_torque_change = constrain(desired_right_torque - g_last_sent_right_torque, -MAX_TORQUE_CHANGE_PER_CYCLE, MAX_TORQUE_CHANGE_PER_CYCLE);
    float left_torque_change = constrain(desired_left_torque - g_last_sent_left_torque, -MAX_TORQUE_CHANGE_PER_CYCLE, MAX_TORQUE_CHANGE_PER_CYCLE);

    float final_right_torque = g_last_sent_right_torque + right_torque_change;
    float final_left_torque = g_last_sent_left_torque + left_torque_change;

    // --- SEND COMMANDS TO ODRIVE ---
    ODRIVE_SERIAL.print("c 0 " + String(final_right_torque, 2) + "\n");
    ODRIVE_SERIAL.print("c 1 " + String(final_left_torque, 2) + "\n");

    g_last_sent_right_torque = final_right_torque;
    g_last_sent_left_torque = final_left_torque;
}
