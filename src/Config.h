#pragma once

#include <Arduino.h>

// --- Tuning Mode ---
#define TUNING_MODE false

// --- UWB Reliability Eval ---
// Logs every incoming UWB packet the instant it arrives over ESP-NOW (NOT
// gated to the 20ms control loop), so packet rate, dropout gaps, and per-
// reading noise can be measured independent of drive state. Doesn't require
// STATE_VELOCITY_AUTO -- just power the cart and wear the tag.
#define UWB_EVAL_LOG false

// --- Raw UWB CSV Log ---
// Emits one CSV row per control cycle with the raw, unfiltered UWB input
// -- for feeding replay_harness.py. Independent of AUTO_DRY_RUN so you can
// run both at once (dry run for eyeballing live, CSV for later replay).

#define RAW_UWB_CSV_LOG false

// Config.h — new flag alongside RAW_UWB_CSV_LOG
#define FULL_STATE_CSV_LOG true

// --- Auto Dry Run ---
#define AUTO_DRY_RUN true

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
constexpr float TORQUE_RAMP_RATE     = 2.0f;

// --- Velocity Control ---
constexpr float MAX_VELOCITY_RPS = 3.0f;
constexpr float VEL_ACCEL_LIMIT  = 1.5f;
constexpr float VEL_DECEL_LIMIT  = 10.0f;

// --- Bearing / Steering ---

// Flip to 1.0f if steering direction reverses
constexpr float STEER_DIR = -1.0f;

// Differential wheel speed per unit of omega (rad/s → RPS).
// Adjust this before changing kp - it's the cleanest authority knob.
constexpr float STEER_MIX_SCALE = 3.0f;

// Forward speed at which full steering authority is granted (RPS).
constexpr float STEER_FULL_AUTHORITY_VEL = 0.3f;

// Max change in omega per 20ms cycle. Prevents snap-turning when static
// friction breaks. Lower = smoother initiation; raise if turns feel sluggish.
constexpr float OMEGA_RAMP_RATE = 0.04f;

constexpr float MIN_WHEEL_VEL = 0.15f; // never let an active wheel go below this
constexpr float TURN_EPSILON    = 0.01f; // |omega_scaled| below this = "going straight", no floor applies

// --- BearingController gains ---
// kp:  proportional - raise if slow to react, lower if oscillates
// ki:  integral - keep small, bearing is noisy
// kd:  derivative - key for "turn earlier + dampen overshoot"
//      raise if still slow to initiate, lower if D-term causes jitter
constexpr float BEARING_KP              = 2.0f;
constexpr float BEARING_KI              = 0.05f;
constexpr float BEARING_KD              = 0.3f;
constexpr float BEARING_KD_FILTER_ALPHA = 0.4f;  // smooths derivative term
constexpr float BEARING_DEADBAND_RAD    = 0.140f; // ±8°
constexpr float BEARING_INTEGRAL_CLAMP  = 0.3f;
constexpr float BEARING_MAX_OMEGA       = 0.8f;

// Bearing input low-pass filter [0,1].
// Raised from 0.1 to 0.3 - the heavy 0.1 filter caused too much lag,
// delaying bearing response by ~0.6s. At 0.3 response is ~0.2s which
// is much better for walking-speed tracking. The derivative term now
// compensates for the noisier but faster signal.
constexpr float BEARING_LPF_ALPHA = 0.3f;

// --- Battery & LVC ---
constexpr bool  ENABLE_LOW_VOLTAGE_CUTOFF = true;
constexpr float LOW_VOLTAGE_CUTOFF        = 32.0f;
constexpr int   LVC_CONSECUTIVE_READINGS  = 10;

// --- Failsafe Timing ---
constexpr unsigned long COMMAND_INTERVAL_MS        = 20;
constexpr unsigned long ESPNOW_FAILSAFE_TIMEOUT_MS = 500;
constexpr unsigned long AUTO_FAILSAFE_TIMEOUT_MS   = 500;
constexpr unsigned long UWB_FAILSAFE_TIMEOUT_MS    = 1000;
// --- Status Broadcast (to tag, via ESP-NOW) ---
constexpr unsigned long STATUS_BROADCAST_INTERVAL_MS = 120;
constexpr unsigned long FAULT_PERSIST_MS = 120;

// --- ESP-NOW Channel Scanning ---
constexpr unsigned long SCAN_INTERVAL_MS    = 150;
constexpr unsigned long SCAN_START_DELAY_MS = 2000;

// --- Derived ---
constexpr float MAX_TORQUE_CHANGE_PER_CYCLE =
    TORQUE_RAMP_RATE * (COMMAND_INTERVAL_MS / 1000.0f);

