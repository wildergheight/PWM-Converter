#pragma once

#include <Arduino.h>

enum ControlState { STATE_TORQUE_ESPNOW, STATE_VELOCITY_AUTO, STATE_BRAKING, STATE_LOW_VOLTAGE_CUTOFF };
enum OdriveQueryType { QUERY_VOLTAGE, QUERY_AXIS0_ERROR, QUERY_AXIS1_ERROR };

// Remote control packet from T-Beam (sizeof = 12 on ESP32)
typedef struct ControlData {
    float throttle;
    float steering;
    bool  button_state;
    bool  auto_mode;
} ControlData;

// UWB ranging packet from Anchor B (packed, sizeof = 10 - distinct from 12)
typedef struct __attribute__((packed)) UWBData {
    float distance_m;    // tag distance from cart centre (m)
    float bearing_rad;   // bearing relative to cart forward; 0 if bearing_valid=false
    bool  valid;         // false → motor driver should brake
    bool  bearing_valid; // false → single-anchor mode, distance only
} UWBData;

// Status alert broadcast to the tag (packed, sizeof = 8 - distinct from UWBData/ControlData)
typedef struct __attribute__((packed)) StatusAlert {
    uint8_t control_state;
    bool    distance_fault;
    bool    bearing_fault;
    bool    lvc_active;
    float   bus_voltage;
} StatusAlert;

extern bool          g_distance_fault;
extern bool          g_bearing_fault;
extern unsigned long g_last_status_broadcast_time;

extern volatile bool g_uwb_data_fresh; // true when a new UWB packet has arrived since last consumed

extern ControlState g_control_state;

extern ControlData              espnowData;
extern volatile unsigned long   g_last_espnow_command_time;
extern int                      g_current_channel;
extern unsigned long            g_last_scan_time;

extern UWBData                  g_uwb_data;
extern volatile unsigned long   g_last_uwb_data_time;

// --- UWB Eval Logging ---
// Filled inside the ESP-NOW recv callback (onDataRecv), printed from
// checkUwbEvalLog() in the main loop -- keeps Serial I/O out of the
// ESP-NOW callback context. No-op overhead if UWB_EVAL_LOG is false.
extern volatile bool          g_uwb_eval_pending;
extern volatile unsigned long g_uwb_eval_t_ms;
extern volatile unsigned long g_uwb_eval_gap_ms;
extern volatile float         g_uwb_eval_distance_m;
extern volatile float         g_uwb_eval_bearing_rad;
extern volatile bool          g_uwb_eval_valid;
extern volatile bool          g_uwb_eval_bearing_valid;

extern String         g_auto_serial_buffer;
extern float          g_auto_right_norm;
extern float          g_auto_left_norm;
extern unsigned long  g_last_auto_command_time;

extern float g_last_sent_right_torque;
extern float g_last_sent_left_torque;
extern float g_current_vel_left;
extern float g_current_vel_right;

extern OdriveQueryType  g_pending_query;
extern OdriveQueryType  g_awaiting_reply;
extern unsigned long    g_last_query_time;
extern bool             g_axis0_fault;
extern bool             g_axis1_fault;

extern float          g_bus_voltage;
extern bool           g_lvc_activated;
extern int            g_lvc_consecutive_count;

extern unsigned long  g_last_command_time;