#pragma once

/*
 * Globals.h
 * Shared types (ControlData, enums) and all global state variables used
 * across modules. Variables are declared extern here and defined once in
 * Globals.cpp.
 */

#include <Arduino.h>

// --- Control State Machine ---
enum ControlState { STATE_TORQUE_ESPNOW, STATE_VELOCITY_AUTO, STATE_BRAKING, STATE_LOW_VOLTAGE_CUTOFF };

// --- ODrive Status Query Rotation ---
enum OdriveQueryType { QUERY_VOLTAGE, QUERY_AXIS0_ERROR, QUERY_AXIS1_ERROR };

// --- ESP-NOW Packet Format ---
typedef struct ControlData {
    float throttle;
    float steering;
    bool button_state; // 0 = off, 1 = on
    bool auto_mode;
} ControlData;

// ---------------------------------------------------------------------------
// Global Variables (defined in Globals.cpp)
// ---------------------------------------------------------------------------

// Control state
extern ControlState g_control_state;

// ESP-NOW
extern ControlData espnowData;
extern volatile unsigned long g_last_espnow_command_time;
extern int g_current_channel;
extern unsigned long g_last_scan_time;

// Automation (USB) serial command parsing
extern String g_auto_serial_buffer;
extern float g_auto_right_norm;
extern float g_auto_left_norm;
extern unsigned long g_last_auto_command_time;

// Torque rate limiter state
extern float g_last_sent_right_torque;
extern float g_last_sent_left_torque;

// Velocity ramp state
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
