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

// --- ODrive Serial Port Configuration ---
#define ODRIVE_SERIAL Serial2
constexpr int ODRIVE_RX_PIN = 16;
constexpr int ODRIVE_TX_PIN = 17;
constexpr long ODRIVE_BAUD_RATE = 115200;

// --- Automation Serial Port (Main USB) ---
#define AUTO_SERIAL Serial
constexpr long AUTO_BAUD_RATE = 115200;

// --- Status LEDs ---
constexpr int RIGHT_LED = 18; // Right motor / axis0 fault indicator
constexpr int LEFT_LED  = 19; // Left motor / axis1 fault indicator
constexpr int BATT_LED  = 21; // Low battery (LVC) indicator

// --- ODrive Status Polling ---
constexpr unsigned long QUERY_INTERVAL_MS = 100;

// --- Torque Control Tuning ---
constexpr float MAX_TORQUE = 30.0;
constexpr float STEERING_SENSITIVITY = 0.5;
constexpr float THROTTLE_EXPO = 2.3;
constexpr float TORQUE_RAMP_RATE = 2.0; // [Nm/sec]

// --- Velocity Control Tuning ---
constexpr float MAX_VELOCITY_RPS = 3.0;
constexpr float VEL_ACCEL_LIMIT = 1.5;          // Slow increase to prevent wheelies
constexpr float VEL_DECEL_LIMIT = 10.0;         // Fast decrease for safety/stopping
constexpr float VEL_STEER_SENSITIVITY = 1.0;    // 1.0 = Steering is instant (unfiltered)

// --- Battery & Low Voltage Cutoff ---
constexpr bool ENABLE_LOW_VOLTAGE_CUTOFF = true;
// IMPORTANT: Set this to a SAFE voltage for your battery pack.
// For a 36V 10S Li-ion pack, 32V (3.2V/cell) is a safe cutoff point.
constexpr float LOW_VOLTAGE_CUTOFF = 32.0;
constexpr unsigned long VOLTAGE_CHECK_INTERVAL_MS = 2000; // unused directly now; query cadence is QUERY_INTERVAL_MS
constexpr int LVC_CONSECUTIVE_READINGS = 10; // Must see low voltage this many times in a row to trigger

// --- Failsafe and Timing ---
constexpr unsigned long COMMAND_INTERVAL_MS = 20;
constexpr unsigned long ESPNOW_FAILSAFE_TIMEOUT_MS = 500; // Fallback to brake after 500ms
constexpr unsigned long AUTO_FAILSAFE_TIMEOUT_MS = 500;   // Fallback to ESP-NOW after 500ms

// Derived: max torque change allowed per 20ms cycle
constexpr float MAX_TORQUE_CHANGE_PER_CYCLE = TORQUE_RAMP_RATE * (COMMAND_INTERVAL_MS / 1000.0f);

// --- ESP-NOW Channel Scanning ---
constexpr unsigned long SCAN_INTERVAL_MS = 150;       // How long to stay on each channel while hunting
constexpr unsigned long SCAN_START_DELAY_MS = 2000;   // Wait this long after last packet before hopping channels
