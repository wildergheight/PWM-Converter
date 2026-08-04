#pragma once

#include <Arduino.h>

// --- Tuning Mode ---
#define TUNING_MODE false

// --- Auto Dry Run ---
// true  = logs computed commands over serial, does NOT send to ODrive.
// false = live operation.
#define AUTO_DRY_RUN false

// --- ODrive Serial ---
#define ODRIVE_SERIAL Serial2
constexpr int  ODRIVE_RX_PIN    = 16;
constexpr int  ODRIVE_TX_PIN    = 17;
constexpr long ODRIVE_BAUD_RATE = 115200;

// --- USB Serial ---
#define AUTO_SERIAL Serial
constexpr long AUTO_BAUD_RATE = 115200;

// --- Status LEDs ---
constexpr int RIGHT_LED = 18;
constexpr int LEFT_LED  = 19;
constexpr int BATT_LED  = 21;

// --- ODrive Polling ---
constexpr unsigned long QUERY_INTERVAL_MS = 100;

// --- Torque Control ---
constexpr float MAX_TORQUE           = 30.0f;
constexpr float STEERING_SENSITIVITY = 0.5f;
constexpr float THROTTLE_EXPO        = 2.3f;
constexpr float TORQUE_RAMP_RATE     = 2.0f; // Nm/s

// --- Velocity Control ---
constexpr float MAX_VELOCITY_RPS = 3.0f;
constexpr float VEL_ACCEL_LIMIT  = 1.5f;
constexpr float VEL_DECEL_LIMIT  = 10.0f;

// --- Bearing / Steering ---

// Flip to 1.0f if steering direction is reversed (depends on which side
// Anchor B is mounted relative to the cart's forward direction).
constexpr float STEER_DIR = -1.0f;

// Converts omega (rad/s) → differential wheel velocity (RPS).
// This is the most impactful steering tuning knob - adjust before kp.
// On a maintained golf fairway start here and come down if it overcorrects.
constexpr float STEER_MIX_SCALE = 3.0f;

// Speed at which full steering authority is granted (RPS).
// Below this, omega scales linearly to zero.
constexpr float STEER_FULL_AUTHORITY_VEL = 0.3f;

// BearingController PID gains.
// kp: (rad/s) per radian of bearing error. Raise if turns feel sluggish,
//     lower first if you see steering oscillation.
// ki: very small - bearing is noisy. Raise only after kp is settled.
// deadband_rad: errors smaller than this produce no steering output.
//     Should sit comfortably above your noise floor (±8° = 0.140 rad).
constexpr float BEARING_KP           = 3.0f;
constexpr float BEARING_KI           = 0.05f;
constexpr float BEARING_DEADBAND_RAD = 0.140f; // ±8°
constexpr float BEARING_INTEGRAL_CLAMP = 0.3f;
constexpr float BEARING_MAX_OMEGA    = 0.8f;   // raised from 0.5 to allow snappier turns

// Bearing low-pass filter coefficient [0,1].
// Lower = heavier filtering, more lag. Higher = more responsive, more noise.
// Raise toward 0.4 if turns feel delayed; lower toward 0.1 if bearing is jumpy.
constexpr float BEARING_LPF_ALPHA = 0.2f;

// --- Battery & LVC ---
constexpr bool  ENABLE_LOW_VOLTAGE_CUTOFF = true;
constexpr float LOW_VOLTAGE_CUTOFF        = 32.0f;
constexpr int   LVC_CONSECUTIVE_READINGS  = 10;

// --- Failsafe Timing ---
constexpr unsigned long COMMAND_INTERVAL_MS        = 20;
constexpr unsigned long ESPNOW_FAILSAFE_TIMEOUT_MS = 500;
constexpr unsigned long AUTO_FAILSAFE_TIMEOUT_MS   = 500;
constexpr unsigned long UWB_FAILSAFE_TIMEOUT_MS    = 1000;

// --- Status Alert (to tag) ---
constexpr unsigned long STATUS_BROADCAST_INTERVAL_MS = 120;
constexpr unsigned long FAULT_PERSIST_MS              = 1000; // error must persist this long before alerting

// --- ESP-NOW Channel Scanning ---
constexpr unsigned long SCAN_INTERVAL_MS    = 150;
constexpr unsigned long SCAN_START_DELAY_MS = 2000;

// --- Derived ---
constexpr float MAX_TORQUE_CHANGE_PER_CYCLE =
    TORQUE_RAMP_RATE * (COMMAND_INTERVAL_MS / 1000.0f);