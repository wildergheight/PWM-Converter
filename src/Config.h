#pragma once

/*
 * Config.h
 * All compile-time constants: pins, tuning parameters, timing, and the
 * TUNING_MODE flag. Nothing in here changes at runtime.
 */

#include <Arduino.h>

// --- Tuning Mode ---
// Set to true when using odrivetool/Python scripts to tune PID gains.
// Disables ALL ESP32 writes to the ODrive so external tools have exclusive control.
// MUST be false for normal cart operation.
#define TUNING_MODE false

// --- Auto Dry Run ---
// true  = STATE_VELOCITY_AUTO logs computed commands over serial but does NOT
//         send them to the ODrive. Use this to validate PID behaviour while
//         pushing the cart by hand.
// false = live operation, commands go to ODrive as normal.
#define AUTO_DRY_RUN true

// --- ODrive Serial Port Configuration ---
#define ODRIVE_SERIAL Serial2
constexpr int ODRIVE_RX_PIN = 16;
constexpr int ODRIVE_TX_PIN = 17;
constexpr long ODRIVE_BAUD_RATE = 115200;

// --- Automation Serial Port (Main USB) ---
#define AUTO_SERIAL Serial
constexpr long AUTO_BAUD_RATE = 115200;

// --- Status LEDs ---
constexpr int RIGHT_LED = 18;
constexpr int LEFT_LED  = 19;
constexpr int BATT_LED  = 21;

// --- ODrive Status Polling ---
constexpr unsigned long QUERY_INTERVAL_MS = 100;

// --- Torque Control Tuning ---
constexpr float MAX_TORQUE = 30.0;
constexpr float STEERING_SENSITIVITY = 0.5;
constexpr float THROTTLE_EXPO = 2.3;
constexpr float TORQUE_RAMP_RATE = 2.0; // [Nm/sec]

// --- Velocity Control Tuning ---
constexpr float MAX_VELOCITY_RPS = 3.0;
constexpr float VEL_ACCEL_LIMIT = 1.5;
constexpr float VEL_DECEL_LIMIT = 10.0;
constexpr float VEL_STEER_SENSITIVITY = 1.0;

// --- Battery & Low Voltage Cutoff ---
constexpr bool ENABLE_LOW_VOLTAGE_CUTOFF = true;
constexpr float LOW_VOLTAGE_CUTOFF = 32.0;
constexpr unsigned long VOLTAGE_CHECK_INTERVAL_MS = 2000;
constexpr int LVC_CONSECUTIVE_READINGS = 10;

// --- Failsafe and Timing ---
constexpr unsigned long COMMAND_INTERVAL_MS = 20;
constexpr unsigned long ESPNOW_FAILSAFE_TIMEOUT_MS = 500;
constexpr unsigned long AUTO_FAILSAFE_TIMEOUT_MS = 500;
constexpr unsigned long UWB_FAILSAFE_TIMEOUT_MS = 1000; // Brake if no UWB packet for 1s

// Derived
constexpr float MAX_TORQUE_CHANGE_PER_CYCLE = TORQUE_RAMP_RATE * (COMMAND_INTERVAL_MS / 1000.0f);

// --- ESP-NOW Channel Scanning ---
constexpr unsigned long SCAN_INTERVAL_MS = 150;
constexpr unsigned long SCAN_START_DELAY_MS = 2000;