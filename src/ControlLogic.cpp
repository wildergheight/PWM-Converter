#include "ControlLogic.h"
#include "Config.h"
#include "OdriveComm.h"

float applyAsymmetricRamp(float target, float current) {
    float delta = target - current;
    float max_change;

    // (If target and current have the same sign and target is larger, we are accelerating)
    if (abs(target) > abs(current)) {
        max_change = VEL_ACCEL_LIMIT * (COMMAND_INTERVAL_MS / 1000.0);
    } else {
        // We are slowing down or crossing zero - use the fast limit
        max_change = VEL_DECEL_LIMIT * (COMMAND_INTERVAL_MS / 1000.0);
    }

    return current + constrain(delta, -max_change, max_change);
}

ControlState determineDesiredState() {
    bool espnow_failsafe = (millis() - g_last_espnow_command_time > ESPNOW_FAILSAFE_TIMEOUT_MS);
    bool auto_override = (millis() - g_last_auto_command_time < AUTO_FAILSAFE_TIMEOUT_MS);

    if (espnow_failsafe || espnowData.button_state) {
        return STATE_BRAKING;
    } else if (ENABLE_LOW_VOLTAGE_CUTOFF && g_lvc_consecutive_count >= LVC_CONSECUTIVE_READINGS) {
        return STATE_LOW_VOLTAGE_CUTOFF;
    } else if (auto_override || espnowData.auto_mode) {
        return STATE_VELOCITY_AUTO;
    } else {
        return STATE_TORQUE_ESPNOW;
    }
}

void handleStateTransition(ControlState desired_state) {
    if (desired_state == g_control_state) return;

#if !TUNING_MODE
    sendOdriveModeTransition(g_control_state, desired_state);

    // When LVC is triggered for the first time
    if (desired_state == STATE_LOW_VOLTAGE_CUTOFF && !g_lvc_activated) {
        g_lvc_activated = true; // Latch the LVC state
        AUTO_SERIAL.println("!!! LOW VOLTAGE CUTOFF ACTIVATED !!!");
        AUTO_SERIAL.println("Voltage: " + String(g_bus_voltage) + "V. System halted.");
        digitalWrite(BATT_LED, HIGH);
    }

    delay(5);
#endif
    // g_control_state is updated by the caller (main loop) regardless of
    // TUNING_MODE, so state tracking stays accurate even when ODrive writes
    // are disabled for bench tuning.
}

void executeControlState() {
#if TUNING_MODE
    return; // odrivetool has exclusive control of the serial line.
#else
    switch (g_control_state) {
        case STATE_LOW_VOLTAGE_CUTOFF:
            // Highest priority state. Brakes are applied and held, unless system is reset.
            sendOdriveBrake();
            if (g_lvc_activated) {
                while (1) { // Halt system
                    sendOdriveBrake();
                    AUTO_SERIAL.print("v 0 0 ");
                    AUTO_SERIAL.println("v 1 0");
                    delay(5);
                }
            }
            break;

        case STATE_BRAKING:
            sendOdriveBrake();
            AUTO_SERIAL.print("v 0 0 ");
            AUTO_SERIAL.println("v 1 0");
            break;

        case STATE_VELOCITY_AUTO: {
            AUTO_SERIAL.print("Auto Right: " + String(espnowData.throttle, 2) + " ");
            AUTO_SERIAL.println("Auto Left: " + String(espnowData.steering, 2));

            if (espnowData.auto_mode) {
                g_auto_right_norm = espnowData.throttle; // THROTTLE IS RIGHT MOTOR IN ESPNOW AUTO MODE
                g_auto_left_norm = espnowData.steering;  // STEERING IS LEFT MOTOR IN ESPNOW AUTO MODE
            }

            float right_vel = g_auto_right_norm * MAX_VELOCITY_RPS;
            float left_vel = g_auto_left_norm * MAX_VELOCITY_RPS;

            g_current_vel_right = applyAsymmetricRamp(right_vel, g_current_vel_right);
            g_current_vel_left = applyAsymmetricRamp(left_vel, g_current_vel_left);

            sendOdriveVelocity(g_current_vel_right, g_current_vel_left);
            break;
        }

        case STATE_TORQUE_ESPNOW: {
            float throttle_input = espnowData.steering;
            float steering_input = espnowData.throttle;

            AUTO_SERIAL.print("Throttle: " + String(espnowData.throttle, 2) + " ");
            AUTO_SERIAL.println("Steering: " + String(espnowData.steering, 2));

            float steering_scale_factor = 1.0 - (STEERING_SENSITIVITY * abs(throttle_input));
            float scaled_steering = steering_input * steering_scale_factor;

            float right_norm = constrain(throttle_input - scaled_steering, -1.0, 1.0);
            float left_norm = constrain(throttle_input + scaled_steering, -1.0, 1.0);

            float curved_right = copysignf(powf(fabsf(right_norm), THROTTLE_EXPO), right_norm);
            float curved_left = copysignf(powf(fabsf(left_norm), THROTTLE_EXPO), left_norm);

            float desired_right_torque = curved_right * MAX_TORQUE;
            float desired_left_torque = curved_left * MAX_TORQUE;

            float right_delta = desired_right_torque - g_last_sent_right_torque;
            float left_delta = desired_left_torque - g_last_sent_left_torque;

            // Ramp only if ABS torque is increasing
            float right_change;
            if (fabsf(desired_right_torque) > fabsf(g_last_sent_right_torque)) {
                right_change = constrain(right_delta, -MAX_TORQUE_CHANGE_PER_CYCLE, MAX_TORQUE_CHANGE_PER_CYCLE);
            } else {
                right_change = right_delta;
            }

            float left_change;
            if (fabsf(desired_left_torque) > fabsf(g_last_sent_left_torque)) {
                left_change = constrain(left_delta, -MAX_TORQUE_CHANGE_PER_CYCLE, MAX_TORQUE_CHANGE_PER_CYCLE);
            } else {
                left_change = left_delta;
            }

            float final_right_torque = g_last_sent_right_torque + right_change;
            float final_left_torque = g_last_sent_left_torque + left_change;

            g_last_sent_right_torque = final_right_torque;
            g_last_sent_left_torque = final_left_torque;

            sendOdriveTorque(final_right_torque, final_left_torque);
            break;
        }
    }
#endif
}
