#include "ControlLogic.h"
#include "Config.h"
#include "OdriveComm.h"
#include "DistanceController.h"

// DistanceController instance - lives here, used only in STATE_VELOCITY_AUTO.
// Config uses the defaults from DistanceControllerConfig (desired_distance=1.5m,
// kp=3.0, ki=0.2, deadband=0.15m). Override fields here before first use if needed.
static DistanceControllerConfig distCfg;
static DistanceController distController(distCfg);
static bool distControllerReady = false; // reset integrator on first entry to auto mode

float applyAsymmetricRamp(float target, float current) {
    float delta = target - current;
    float max_change;

    if (abs(target) > abs(current)) {
        max_change = VEL_ACCEL_LIMIT * (COMMAND_INTERVAL_MS / 1000.0);
    } else {
        max_change = VEL_DECEL_LIMIT * (COMMAND_INTERVAL_MS / 1000.0);
    }

    return current + constrain(delta, -max_change, max_change);
}

ControlState determineDesiredState() {
    bool espnow_failsafe  = (millis() - g_last_espnow_command_time > ESPNOW_FAILSAFE_TIMEOUT_MS);
    bool auto_override    = (millis() - g_last_auto_command_time < AUTO_FAILSAFE_TIMEOUT_MS);
    bool uwb_data_fresh   = (millis() - g_last_uwb_data_time < UWB_FAILSAFE_TIMEOUT_MS);

    if (espnow_failsafe || espnowData.button_state) {
        return STATE_BRAKING;
    } else if (ENABLE_LOW_VOLTAGE_CUTOFF && g_lvc_consecutive_count >= LVC_CONSECUTIVE_READINGS) {
        return STATE_LOW_VOLTAGE_CUTOFF;
    } else if (auto_override || espnowData.auto_mode) {
        // In auto mode, UWB data must be fresh — otherwise brake rather than
        // drive blind. The UWB link loss is a more pressing stop condition than
        // the remote link loss since it means we can't track the operator at all.
        if (espnowData.auto_mode && !uwb_data_fresh) {
            AUTO_SERIAL.println("AUTO: UWB data stale - braking");
            return STATE_BRAKING;
        }
        return STATE_VELOCITY_AUTO;
    } else {
        return STATE_TORQUE_ESPNOW;
    }
}

void handleStateTransition(ControlState desired_state) {
    if (desired_state == g_control_state) return;

    // Reset DistanceController integrator whenever we leave STATE_VELOCITY_AUTO
    // so stale integral windup doesn't carry over into the next auto session.
    if (g_control_state == STATE_VELOCITY_AUTO && desired_state != STATE_VELOCITY_AUTO) {
        distController.reset();
        distControllerReady = false;
        g_current_vel_left  = 0.0f;
        g_current_vel_right = 0.0f;
    }

#if !TUNING_MODE
    sendOdriveModeTransition(g_control_state, desired_state);

    if (desired_state == STATE_LOW_VOLTAGE_CUTOFF && !g_lvc_activated) {
        g_lvc_activated = true;
        AUTO_SERIAL.println("!!! LOW VOLTAGE CUTOFF ACTIVATED !!!");
        AUTO_SERIAL.println("Voltage: " + String(g_bus_voltage) + "V. System halted.");
        digitalWrite(BATT_LED, HIGH);
    }

    delay(5);
#endif
}

void executeControlState() {
    switch (g_control_state) {

        case STATE_LOW_VOLTAGE_CUTOFF:
#if !TUNING_MODE
            sendOdriveBrake();
            if (g_lvc_activated) {
                while (1) {
                    sendOdriveBrake();
                    AUTO_SERIAL.println("LVC HALT: v 0 0");
                    delay(5);
                }
            }
#endif
            break;

        case STATE_BRAKING:
#if !TUNING_MODE
            sendOdriveBrake();
#endif
            AUTO_SERIAL.println("BRAKE: v 0 0");
            break;

        case STATE_VELOCITY_AUTO: {
            // --- Reset integrator on first entry ---
            if (!distControllerReady) {
                distController.reset();
                distControllerReady = true;
            }

            // --- USB serial override (for manual bench testing) ---
            if (millis() - g_last_auto_command_time < AUTO_FAILSAFE_TIMEOUT_MS) {
                float right_vel = g_auto_right_norm * MAX_VELOCITY_RPS;
                float left_vel  = g_auto_left_norm  * MAX_VELOCITY_RPS;
                g_current_vel_right = applyAsymmetricRamp(right_vel, g_current_vel_right);
                g_current_vel_left  = applyAsymmetricRamp(left_vel,  g_current_vel_left);

#if !TUNING_MODE && !AUTO_DRY_RUN
                sendOdriveVelocity(g_current_vel_right, g_current_vel_left);
#endif
                AUTO_SERIAL.println("USB_AUTO | R=" + String(g_current_vel_right, 2)
                                  + " L=" + String(g_current_vel_left, 2));
                break;
            }

            // --- UWB distance control ---
            float dist    = g_uwb_data.distance_m;
            float bearing = g_uwb_data.bearing_rad;

            // DistanceController gives us the target forward velocity.
            // bearing is available here but not used until Phase B tuning.
            float v_forward = distController.update(dist);

            // Both wheels get the same forward velocity (no steering yet).
            float target_vel = v_forward;
            g_current_vel_right = applyAsymmetricRamp(target_vel, g_current_vel_right);
            g_current_vel_left  = applyAsymmetricRamp(target_vel, g_current_vel_left);

#if AUTO_DRY_RUN
            // Dry run: log what WOULD go to the ODrive, don't send it.
            AUTO_SERIAL.print("DRY_RUN | dist=");
            AUTO_SERIAL.print(dist, 3);
            AUTO_SERIAL.print("m | bearing=");
            AUTO_SERIAL.print(bearing * 180.0f / PI, 1);
            AUTO_SERIAL.print("deg | v_fwd=");
            AUTO_SERIAL.print(v_forward, 3);
            AUTO_SERIAL.print(" | v_r=");
            AUTO_SERIAL.print(g_current_vel_right, 3);
            AUTO_SERIAL.print(" | v_l=");
            AUTO_SERIAL.print(g_current_vel_left, 3);
            AUTO_SERIAL.print(" | err=");
            AUTO_SERIAL.print(distController.getLastError(), 3);
            AUTO_SERIAL.print(" | intg=");
            AUTO_SERIAL.println(distController.getIntegral(), 3);
#else
    #if !TUNING_MODE
            sendOdriveVelocity(g_current_vel_right, g_current_vel_left);
    #endif
#endif
            break;
        }

        case STATE_TORQUE_ESPNOW: {
#if TUNING_MODE
            break;
#else
            float throttle_input = espnowData.steering;
            float steering_input = espnowData.throttle;

            float steering_scale_factor = 1.0 - (STEERING_SENSITIVITY * abs(throttle_input));
            float scaled_steering = steering_input * steering_scale_factor;

            float right_norm = constrain(throttle_input - scaled_steering, -1.0, 1.0);
            float left_norm  = constrain(throttle_input + scaled_steering, -1.0, 1.0);

            float curved_right = copysignf(powf(fabsf(right_norm), THROTTLE_EXPO), right_norm);
            float curved_left  = copysignf(powf(fabsf(left_norm),  THROTTLE_EXPO), left_norm);

            float desired_right_torque = curved_right * MAX_TORQUE;
            float desired_left_torque  = curved_left  * MAX_TORQUE;

            float right_delta = desired_right_torque - g_last_sent_right_torque;
            float left_delta  = desired_left_torque  - g_last_sent_left_torque;

            float right_change = (fabsf(desired_right_torque) > fabsf(g_last_sent_right_torque))
                ? constrain(right_delta, -MAX_TORQUE_CHANGE_PER_CYCLE, MAX_TORQUE_CHANGE_PER_CYCLE)
                : right_delta;

            float left_change = (fabsf(desired_left_torque) > fabsf(g_last_sent_left_torque))
                ? constrain(left_delta, -MAX_TORQUE_CHANGE_PER_CYCLE, MAX_TORQUE_CHANGE_PER_CYCLE)
                : left_delta;

            float final_right_torque = g_last_sent_right_torque + right_change;
            float final_left_torque  = g_last_sent_left_torque  + left_change;

            g_last_sent_right_torque = final_right_torque;
            g_last_sent_left_torque  = final_left_torque;

            sendOdriveTorque(final_right_torque, final_left_torque);
            break;
#endif
        }
    }
}