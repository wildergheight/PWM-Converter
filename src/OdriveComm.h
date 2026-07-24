#pragma once

/*
 * OdriveComm.h
 * Everything that talks to the ODrive over ODRIVE_SERIAL:
 *  - round-robin status polling (voltage + axis error codes)
 *  - control-mode transition commands
 *  - torque / velocity command output
 *
 * All ODRIVE_SERIAL writes are gated behind !TUNING_MODE at the call site
 * (in ControlLogic.cpp / main.cpp), so bench tuning with odrivetool never
 * collides with ESP32 writes.
 */

#include <Arduino.h>
#include "Globals.h"

// Polls the ODrive in round-robin fashion (vbus_voltage, axis0.error, axis1.error),
// one query per call, and parses whatever response(s) are available.
// Updates g_bus_voltage, g_lvc_consecutive_count, g_axis0_fault, g_axis1_fault.
void checkODriveStatus();

// Sends the appropriate control_mode / input_mode write commands to the ODrive
// when transitioning between torque and velocity control. Only sends anything
// when the transition actually changes control mode.
void sendOdriveModeTransition(ControlState from_state, ControlState to_state);

// Sends the brake command (zero velocity) to both axes.
void sendOdriveBrake();

// Sends velocity commands to both axes (used in STATE_VELOCITY_AUTO).
void sendOdriveVelocity(float right_vel, float left_vel);

// Sends torque commands to both axes (used in STATE_TORQUE_ESPNOW).
void sendOdriveTorque(float right_torque, float left_torque);
