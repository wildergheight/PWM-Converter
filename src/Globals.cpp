#include "Globals.h"

ControlState g_control_state = STATE_BRAKING;

ControlData espnowData = {};
volatile unsigned long g_last_espnow_command_time = 0;
int g_current_channel = 1;
unsigned long g_last_scan_time = 0;

UWBData g_uwb_data = {0.0f, 0.0f, false};
volatile unsigned long g_last_uwb_data_time = 0;

String g_auto_serial_buffer = "";
float g_auto_right_norm = 0.0;
float g_auto_left_norm = 0.0;
unsigned long g_last_auto_command_time = 0;

float g_last_sent_right_torque = 0.0;
float g_last_sent_left_torque = 0.0;

float g_current_vel_left = 0.0;
float g_current_vel_right = 0.0;

OdriveQueryType g_pending_query  = QUERY_VOLTAGE;
OdriveQueryType g_awaiting_reply = QUERY_VOLTAGE;
unsigned long g_last_query_time = 0;
bool g_axis0_fault = false;
bool g_axis1_fault = false;

float g_bus_voltage = 0.0;
bool g_lvc_activated = false;
int g_lvc_consecutive_count = 0;

unsigned long g_last_command_time = 0;

bool          g_distance_fault = false;
bool          g_bearing_fault  = false;
unsigned long g_last_status_broadcast_time = 0;