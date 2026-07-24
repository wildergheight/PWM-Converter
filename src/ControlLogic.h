#pragma once

/*
 * ControlLogic.h
 * The state machine itself: deciding which ControlState we should be in
 * each cycle, handling transitions, and executing the per-state behavior
 * (braking / LVC halt / velocity-auto / torque-espnow).
 */

#include <Arduino.h>
#include "Globals.h"

// Applies an asymmetric accel/decel ramp: slow to speed up, fast to slow down.
float applyAsymmetricRamp(float target, float current);

// Looks at failsafe timers, LVC latch, and ESP-NOW/auto override flags to
// decide which ControlState we should be in this cycle.
ControlState determineDesiredState();

// Handles any bookkeeping needed when transitioning between states
// (ODrive mode switch, LVC first-trigger latch/log). Does not change
// g_control_state itself -- caller does that after this returns.
void handleStateTransition(ControlState desired_state);

// Executes the ODrive command(s) appropriate for the current g_control_state.
void executeControlState();
