#pragma once

/*
 * Globals.h
 * Shared types (ControlData, UWBData, enums) and all global state variables
 * used across modules.
 */

#include <Arduino.h>

// --- Control State Machine ---
enum ControlState { STATE_TORQUE_ESPNOW, STATE_VELOCITY_AUTO, STATE_BRAKING, STATE_LOW_VOLTAGE_CUTOFF };

// --- ODrive Status Query Rotation ---
enum OdriveQueryType { QUERY_VOLTAGE, QUERY_AXIS0_ERROR, QUERY_AXIS1_ERROR };

// --- ESP-NOW Packet: remote control (T-Beam) ---
// sizeof(ControlData) = 12 on ESP32 (2x float + 2x bool, padded to 4-byte boundary)
typedef struct ControlData {
    float throttle;
    float steering;
    bool button_state;
    bool auto_mode;
} ControlData;

// --- ESP-NOW Packet: UWB ranging data (Anchor B) ---
// __attribute__((packed)) gives sizeof(UWBData) = 9, distinct from
// sizeof(ControlData) = 12, so onDataRecv can route by packet length alone.
typedef struct __attribute__((packed)) UWBData {
    float distance_m;   // tag distance from cart centre (m)
    float bearing_rad;  // bearing: positive = tag toward Anchor B side
    bool  valid;        // false if Anchor B's trilateration failed
} UWBData;

// ---------------------------------------------------------------------------
// Global Variables (defined in Globals.cpp)
// ---------------------------------------------------------------------------

extern ControlState g_control_state;

// ESP-NOW remote
extern ControlData espnowData;
extern volatile unsigned long g_last_espnow_command_time;
extern int g_current_channel;
extern unsigned long g_last_scan_time;

// ESP-NOW UWB
extern UWBData g_uwb_data;
extern volatile unsigned long g_last_uwb_data_time;

// Automation (USB) serial
extern String g_auto_serial_buffer;
extern float g_auto_right_norm;
extern float g_auto_left_norm;
extern unsigned long g_last_auto_command_time;

// Torque rate limiter
extern float g_last_sent_right_torque;
extern float g_last_sent_left_torque;

// Velocity ramp
extern float g_current_vel_left;
extern float g_current_vel_right;

// ODrive status polling
extern OdriveQueryType g_pending_query;
extern OdriveQueryType g_awaiting_reply;
extern unsigned long g_last_query_time;
extern bool g_axis0_fault;
extern bool g_axis1_fault;

// Voltage & LVC
extern float g_bus_voltage;
extern bool g_lvc_activated;
extern int g_lvc_consecutive_count;

// Main loop timing
extern unsigned long g_last_command_time;