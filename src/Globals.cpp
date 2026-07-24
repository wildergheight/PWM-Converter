#include "Globals.h"

// Control state
ControlState g_control_state = STATE_BRAKING; // Start in BRAKING state for safety

// ESP-NOW
ControlData espnowData = {};
volatile unsigned long g_last_espnow_command_time = 0;
int g_current_channel = 1;
unsigned long g_last_scan_time = 0;

// Automation (USB) serial command parsing
String g_auto_serial_buffer = "";
float g_auto_right_norm = 0.0;
float g_auto_left_norm = 0.0;
unsigned long g_last_auto_command_time = 0;

// Torque rate limiter state
float g_last_sent_right_torque = 0.0;
float g_last_sent_left_torque = 0.0;

// Velocity ramp state
float g_current_vel_left = 0.0;
float g_current_vel_right = 0.0;

// ODrive status polling
OdriveQueryType g_pending_query = QUERY_VOLTAGE;   // What to send NEXT
OdriveQueryType g_awaiting_reply = QUERY_VOLTAGE;  // What we're waiting a reply FOR
unsigned long g_last_query_time = 0;
bool g_axis0_fault = false;
bool g_axis1_fault = false;

// Voltage & LVC
float g_bus_voltage = 0.0;
bool g_lvc_activated = false;
int g_lvc_consecutive_count = 0;

// Main loop timing
unsigned long g_last_command_time = 0;
