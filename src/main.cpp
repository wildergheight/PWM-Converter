/*
 * ESP32 PWM/Serial to ODrive Serial (ASCII) Converter
 * * Author: Gemini
 * * Date: August 9, 2025
 * * Version: 8.0 (Velocity Control for Automation)
 * *
 * * Description:
 * This version uses two different control modes. Manual RC control uses torque
 * with an exponential curve and rate limiter for a responsive feel. Automated
 * control (via USB) uses velocity control for stable, predictable movement.
 * The ESP32 automatically switches the ODrive's control mode based on input.
 *
 * Control Priority & Mode:
 * 1. RC Brake/Failsafe (Highest): Velocity Control (0 vel)
 * 2. Automation Serial Command (USB): Velocity Control
 * 3. RC Control (Lowest): Torque Control
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

// --- Automation Serial Port (Main USB) ---
#define AUTO_SERIAL Serial
const long AUTO_BAUD_RATE = 115200;

// --- Control & Timing Configuration ---
const unsigned int DEFAULT_PULSE_US = 1500;
const unsigned int MIN_PULSE_US = 1000;
const unsigned int MAX_PULSE_US = 2000;
const int PULSE_DEADZONE_US = 10;
const unsigned int BRAKE_ON_THRESHOLD_US = 1500;

// --- Tuning Parameters ---
// FOR RC (Torque) MODE:
const float MAX_TORQUE = 30.0;
const float STEERING_SENSITIVITY = 0.5;
const float THROTTLE_EXPO = 2.0;
const float MAX_TORQUE_CHANGE_PER_CYCLE = 1.0;
// FOR AUTOMATION (Velocity) MODE:
const float MAX_VELOCITY_RPS = 2.0; // Revolutions Per Second

// --- Failsafe and Timing ---
const unsigned long COMMAND_INTERVAL_MS = 20;
const unsigned long RC_FAILSAFE_TIMEOUT_US = 100000; // 100 ms
const unsigned long AUTO_FAILSAFE_TIMEOUT_MS = 500;  // Fallback to RC after 500ms

//================================================================================
// Global Variables
//================================================================================

// --- State Machine for Control ---
enum ControlState { STATE_TORQUE_RC, STATE_VELOCITY_AUTO, STATE_BRAKING };
ControlState g_control_state = STATE_TORQUE_RC;

// --- PWM Input Variables ---
volatile unsigned long g_pulse_throttle_start_time, g_pulse_steering_start_time, g_pulse_brake_start_time;
volatile unsigned int g_pulse_throttle_width_us = DEFAULT_PULSE_US;
volatile unsigned int g_pulse_steering_width_us = DEFAULT_PULSE_US;
volatile unsigned int g_pulse_brake_width_us = MIN_PULSE_US;
volatile unsigned long g_last_pulse_throttle_update, g_last_pulse_steering_update, g_last_pulse_brake_update;

// --- Automation Command Variables ---
String g_auto_serial_buffer = "";
float g_auto_right_norm = 0.0;
float g_auto_left_norm = 0.0;
unsigned long g_last_auto_command_time = 0;

// --- Rate Limiter State (for Torque mode only) ---
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
// Helper Function
//================================================================================
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//================================================================================
// Automation Serial Parsing
//================================================================================
void parseAutoCommand(String cmd) {
    cmd.trim();
    int r_idx = cmd.indexOf('R');
    int l_idx = cmd.indexOf('L');
    int comma_idx = cmd.indexOf(',');

    if (r_idx != -1 && l_idx != -1 && comma_idx != -1) {
        String r_val_str = cmd.substring(r_idx + 2, comma_idx);
        String l_val_str = cmd.substring(l_idx + 2);
        
        g_auto_right_norm = constrain(r_val_str.toFloat(), -1.0, 1.0);
        g_auto_left_norm = constrain(l_val_str.toFloat(), -1.0, 1.0);
        g_last_auto_command_time = millis();
    }
}

void checkAutoSerial() {
    while (AUTO_SERIAL.available()) {
        char c = AUTO_SERIAL.read();
        if (c == '\n') {
            parseAutoCommand(g_auto_serial_buffer);
            g_auto_serial_buffer = "";
        } else {
            g_auto_serial_buffer += c;
        }
    }
}

//================================================================================
// Main Program
//================================================================================
void setup() {
    AUTO_SERIAL.begin(AUTO_BAUD_RATE);
    ODRIVE_SERIAL.begin(ODRIVE_BAUD_RATE, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);
    
    pinMode(THROTTLE_PIN, INPUT_PULLUP);
    pinMode(STEERING_PIN, INPUT_PULLUP);
    pinMode(BRAKE_PIN, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), isr_throttle, CHANGE);
    attachInterrupt(digitalPinToInterrupt(STEERING_PIN), isr_steering, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BRAKE_PIN), isr_brake, CHANGE);
    
    AUTO_SERIAL.println("ESP32 ODrive Bridge v8.0 Ready.");
}

void loop() {
    checkAutoSerial();

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
    } else if (millis() - g_last_auto_command_time < AUTO_FAILSAFE_TIMEOUT_MS) {
        desired_state = STATE_VELOCITY_AUTO;
    } else {
        desired_state = STATE_TORQUE_RC;
    }

    // --- HANDLE ODRIVE MODE TRANSITIONS ---
    if (desired_state != g_control_state) {
        // To BRAKING or AUTO from TORQUE_RC -> Switch to Velocity Control
        if ((desired_state == STATE_BRAKING || desired_state == STATE_VELOCITY_AUTO) && g_control_state == STATE_TORQUE_RC) {
            AUTO_SERIAL.println("STATE: Switching ODrive to VELOCITY_CONTROL");
            ODRIVE_SERIAL.println("w 0.controller.config.control_mode 2");
            ODRIVE_SERIAL.println("w 1.controller.config.control_mode 2");
        } 
        // To TORQUE_RC from any other state -> Switch to Torque Control
        else if (desired_state == STATE_TORQUE_RC && g_control_state != STATE_TORQUE_RC) {
            AUTO_SERIAL.println("STATE: Switching ODrive to TORQUE_CONTROL");
            ODRIVE_SERIAL.println("w 0.controller.config.control_mode 1");
            ODRIVE_SERIAL.println("w 1.controller.config.control_mode 1");
        }
        
        // Reset torque rate limiter when leaving torque mode
        if (g_control_state == STATE_TORQUE_RC) {
            g_last_sent_right_torque = 0.0;
            g_last_sent_left_torque = 0.0;
        }

        g_control_state = desired_state;
        delay(5);
    }

    // --- EXECUTE ACTION BASED ON STATE ---
    switch(g_control_state) {
        case STATE_BRAKING:
            ODRIVE_SERIAL.println("v 0 0");
            ODRIVE_SERIAL.println("v 1 0");
            break;

        case STATE_VELOCITY_AUTO:
            {
                float right_vel = g_auto_right_norm * MAX_VELOCITY_RPS;
                float left_vel = g_auto_left_norm * MAX_VELOCITY_RPS;
                ODRIVE_SERIAL.print("v 0 " + String(right_vel, 2) + "\n");
                ODRIVE_SERIAL.print("v 1 " + String(left_vel, 2) + "\n");
            }
            break;

        case STATE_TORQUE_RC:
            {
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

                float right_norm = constrain(throttle_input - scaled_steering, -1.0, 1.0);
                float left_norm = constrain(throttle_input + scaled_steering, -1.0, 1.0);

                float curved_right = copysignf(powf(fabsf(right_norm), THROTTLE_EXPO), right_norm);
                float curved_left = copysignf(powf(fabsf(left_norm), THROTTLE_EXPO), left_norm);

                float desired_right_torque = curved_right * MAX_TORQUE;
                float desired_left_torque = curved_left * MAX_TORQUE;

                float right_torque_change = constrain(desired_right_torque - g_last_sent_right_torque, -MAX_TORQUE_CHANGE_PER_CYCLE, MAX_TORQUE_CHANGE_PER_CYCLE);
                float left_torque_change = constrain(desired_left_torque - g_last_sent_left_torque, -MAX_TORQUE_CHANGE_PER_CYCLE, MAX_TORQUE_CHANGE_PER_CYCLE);

                float final_right_torque = g_last_sent_right_torque + right_torque_change;
                float final_left_torque = g_last_sent_left_torque + left_torque_change;

                ODRIVE_SERIAL.print("c 0 " + String(final_right_torque, 2) + "\n");
                ODRIVE_SERIAL.print("c 1 " + String(final_left_torque, 2) + "\n");

                g_last_sent_right_torque = final_right_torque;
                g_last_sent_left_torque = final_left_torque;
            }
            break;
    }
}
