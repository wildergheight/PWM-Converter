#include "ControlLogic.h"
#include "Config.h"
#include "OdriveComm.h"
#include "DistanceController.h"

// kp lowered from 3.0 to 2.0 to reduce overshoot/rubber-banding.
// At kp=3.0 a 0.5m error commanded 1.5 RPS which was enough to overshoot
// the setpoint, triggering the no-reverse clamp repeatedly. At kp=2.0 the
// same error gives 1.0 RPS - smoother approach, less oscillation.
// Raise kp again if the cart feels too sluggish to catch up.
static DistanceController distController = []() {
    DistanceControllerConfig cfg;
    cfg.kp = 2.0f; // only override what differs from defaults
    return DistanceController(cfg);
}();
static bool distControllerReady = false;

float applyAsymmetricRamp(float target, float current) {
    float delta = target - current;
    float max_change = (fabsf(target) > fabsf(current))
        ? VEL_ACCEL_LIMIT * (COMMAND_INTERVAL_MS / 1000.0f)
        : VEL_DECEL_LIMIT * (COMMAND_INTERVAL_MS / 1000.0f);
    return current + constrain(delta, -max_change, max_change);
}

ControlState determineDesiredState() {
    bool espnow_failsafe = (millis() - g_last_espnow_command_time > ESPNOW_FAILSAFE_TIMEOUT_MS);
    bool auto_override   = (millis() - g_last_auto_command_time   < AUTO_FAILSAFE_TIMEOUT_MS);
    bool uwb_fresh       = (millis() - g_last_uwb_data_time       < UWB_FAILSAFE_TIMEOUT_MS);

    if (espnow_failsafe || espnowData.button_state) {
        return STATE_BRAKING;
    } else if (ENABLE_LOW_VOLTAGE_CUTOFF && g_lvc_consecutive_count >= LVC_CONSECUTIVE_READINGS) {
        return STATE_LOW_VOLTAGE_CUTOFF;
    } else if (auto_override || espnowData.auto_mode) {
        if (espnowData.auto_mode && !uwb_fresh) {
            AUTO_SERIAL.println("AUTO: UWB stale - braking");
            return STATE_BRAKING;
        }
        // Also brake if Anchor B explicitly marked data invalid
        if (espnowData.auto_mode && !g_uwb_data.valid) {
            AUTO_SERIAL.println("AUTO: UWB invalid - braking");
            return STATE_BRAKING;
        }
        return STATE_VELOCITY_AUTO;
    } else {
        return STATE_TORQUE_ESPNOW;
    }
}

void handleStateTransition(ControlState desired_state) {
    if (desired_state == g_control_state) return;

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
                while (1) { sendOdriveBrake(); delay(5); }
            }
#endif
            break;

        case STATE_BRAKING:
#if !TUNING_MODE
            sendOdriveBrake();
#endif
            AUTO_SERIAL.println("BRAKE");
            break;

        case STATE_VELOCITY_AUTO: {
            if (!distControllerReady) {
                distController.reset();
                distControllerReady = true;
            }

            // USB serial override (bench testing)
            if (millis() - g_last_auto_command_time < AUTO_FAILSAFE_TIMEOUT_MS) {
                float right_vel = g_auto_right_norm * MAX_VELOCITY_RPS;
                float left_vel  = g_auto_left_norm  * MAX_VELOCITY_RPS;
                g_current_vel_right = applyAsymmetricRamp(right_vel, g_current_vel_right);
                g_current_vel_left  = applyAsymmetricRamp(left_vel,  g_current_vel_left);
#if !TUNING_MODE && !AUTO_DRY_RUN
                sendOdriveVelocity(-g_current_vel_right, g_current_vel_left);
#endif
                AUTO_SERIAL.println("USB_AUTO | R=" + String(-g_current_vel_right, 2)
                                  + " L=" + String(g_current_vel_left, 2));
                break;
            }

            // --- UWB distance control ---
            float dist    = g_uwb_data.distance_m;
            float bearing = g_uwb_data.bearing_rad;

            float v_forward = distController.update(dist);

            // No reverse in auto mode: if too close, stop and let the
            // integrator decay rather than accumulating negative windup.
            // The DistanceController's deadband decay handles this when
            // we're near setpoint; when we're clearly over it (v_forward
            // meaningfully negative), nudge the integrator toward zero
            // faster so recovery isn't sluggish when the operator moves away.
            if (v_forward < 0.0f) {
                distController.reset(); // clear windup when pinned to floor
                v_forward = 0.0f;
            }

            g_current_vel_right = applyAsymmetricRamp(v_forward, g_current_vel_right);
            g_current_vel_left  = applyAsymmetricRamp(v_forward, g_current_vel_left);

#if AUTO_DRY_RUN
            AUTO_SERIAL.print("DRY_RUN | dist=");
            AUTO_SERIAL.print(dist, 3);
            AUTO_SERIAL.print("m | brg=");
            AUTO_SERIAL.print(bearing * 180.0f / PI, 1);
            AUTO_SERIAL.print("deg");
            if (!g_uwb_data.bearing_valid) AUTO_SERIAL.print("(single-anchor)");
            AUTO_SERIAL.print(" | v_fwd=");
            AUTO_SERIAL.print(v_forward, 3);
            AUTO_SERIAL.print(" | v_r=");
            AUTO_SERIAL.print(-g_current_vel_right, 3);
            AUTO_SERIAL.print(" | v_l=");
            AUTO_SERIAL.print(g_current_vel_left, 3);
            AUTO_SERIAL.print(" | err=");
            AUTO_SERIAL.print(distController.getLastError(), 3);
            AUTO_SERIAL.print(" | intg=");
            AUTO_SERIAL.println(distController.getIntegral(), 3);
#else
    #if !TUNING_MODE
            sendOdriveVelocity(-g_current_vel_right, g_current_vel_left);
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

            float steering_scale_factor = 1.0f - (STEERING_SENSITIVITY * fabsf(throttle_input));
            float scaled_steering = steering_input * steering_scale_factor;

            float right_norm = constrain(throttle_input - scaled_steering, -1.0f, 1.0f);
            float left_norm  = constrain(throttle_input + scaled_steering, -1.0f, 1.0f);

            float curved_right = copysignf(powf(fabsf(right_norm), THROTTLE_EXPO), right_norm);
            float curved_left  = copysignf(powf(fabsf(left_norm),  THROTTLE_EXPO), left_norm);

            float desired_right = curved_right * MAX_TORQUE;
            float desired_left  = curved_left  * MAX_TORQUE;

            float right_delta = desired_right - g_last_sent_right_torque;
            float left_delta  = desired_left  - g_last_sent_left_torque;

            float right_change = (fabsf(desired_right) > fabsf(g_last_sent_right_torque))
                ? constrain(right_delta, -MAX_TORQUE_CHANGE_PER_CYCLE, MAX_TORQUE_CHANGE_PER_CYCLE)
                : right_delta;
            float left_change = (fabsf(desired_left) > fabsf(g_last_sent_left_torque))
                ? constrain(left_delta, -MAX_TORQUE_CHANGE_PER_CYCLE, MAX_TORQUE_CHANGE_PER_CYCLE)
                : left_delta;

            g_last_sent_right_torque += right_change;
            g_last_sent_left_torque  += left_change;

            sendOdriveTorque(g_last_sent_right_torque, g_last_sent_left_torque);
            break;
#endif
        }
    }
}