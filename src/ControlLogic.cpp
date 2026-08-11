#include "ControlLogic.h"
#include "Config.h"
#include "OdriveComm.h"
#include "DistanceController.h"
#include "Bearingcontroller.h"

// --- DistanceController ---
static DistanceController distController = []() {
    DistanceControllerConfig cfg;
    cfg.kp = 2.0f;
    return DistanceController(cfg);
}();

// --- BearingController ---
static BearingController bearingController = []() {
    BearingControllerConfig cfg;
    cfg.kp              = BEARING_KP;
    cfg.ki              = BEARING_KI;
    cfg.kd              = BEARING_KD;
    cfg.kd_filter_alpha = BEARING_KD_FILTER_ALPHA;
    cfg.deadband_rad    = BEARING_DEADBAND_RAD;
    cfg.integral_clamp  = BEARING_INTEGRAL_CLAMP;
    cfg.max_omega       = BEARING_MAX_OMEGA;
    return BearingController(cfg);
}();

// --- State ---
static float g_bearing_filtered   = 0.0f;
static bool  g_bearing_lpf_ready  = false;
static float g_omega_prev         = 0.0f; // for omega ramp
static bool  distControllerReady  = false;
static bool  bearingControllerReady = false;

//================================================================================
// Helpers
//================================================================================

float applyAsymmetricRamp(float target, float current) {
    float delta = target - current;
    float max_change = (fabsf(target) > fabsf(current))
        ? VEL_ACCEL_LIMIT * (COMMAND_INTERVAL_MS / 1000.0f)
        : VEL_DECEL_LIMIT * (COMMAND_INTERVAL_MS / 1000.0f);
    return current + constrain(delta, -max_change, max_change);
}

//================================================================================
// State machine
//================================================================================

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
        bearingController.reset();
        distControllerReady    = false;
        bearingControllerReady = false;
        g_bearing_lpf_ready    = false;
        g_bearing_filtered     = 0.0f;
        g_omega_prev           = 0.0f;
        g_current_vel_left     = 0.0f;
        g_current_vel_right    = 0.0f;
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

//================================================================================
// Execute
//================================================================================

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
            if (!bearingControllerReady) {
                bearingController.reset();
                g_omega_prev = 0.0f;
                bearingControllerReady = true;
            }

            // --- USB serial override ---
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

            // --- Distance PID → v_forward ---
            float dist = g_uwb_data.distance_m;
            float v_forward = distController.update(dist);
            if (v_forward < 0.0f) {
                distController.reset();
                v_forward = 0.0f;
            }

            #if RAW_UWB_CSV_LOG
                AUTO_SERIAL.print("CSV,");
                AUTO_SERIAL.print(millis());              AUTO_SERIAL.print(",");
                AUTO_SERIAL.print(dist, 4);                AUTO_SERIAL.print(",");
                AUTO_SERIAL.print(g_uwb_data.bearing_rad, 5); AUTO_SERIAL.print(",");
                AUTO_SERIAL.println(g_uwb_data.bearing_valid ? 1 : 0);
            #endif

            // --- Bearing LPF ---
            if (g_uwb_data.bearing_valid) {
                float bearing_raw = g_uwb_data.bearing_rad;
                if (!g_bearing_lpf_ready) {
                    g_bearing_filtered = bearing_raw;
                    g_bearing_lpf_ready = true;
                } else {
                    g_bearing_filtered = BEARING_LPF_ALPHA * bearing_raw
                                       + (1.0f - BEARING_LPF_ALPHA) * g_bearing_filtered;
                }
            } else {
                g_bearing_filtered *= 0.9f;
                g_bearing_lpf_ready = false;
            }

            // --- Bearing PID → omega ---
            float current_speed = (g_current_vel_right + g_current_vel_left) / 2.0f;
            float authority = constrain(current_speed / STEER_FULL_AUTHORITY_VEL, 0.0f, 1.0f);

            float omega = 0.0f;
            if (g_uwb_data.bearing_valid) {
                omega = bearingController.update(g_bearing_filtered, authority);
            } else {
                bearingController.reset();
            }

            // Scale by authority and apply STEER_DIR
            float omega_target = omega * authority * STEER_DIR;

            // --- Omega ramp: prevents snap-turning when stiction breaks ---
            // The cart may resist turning until friction is overcome, then
            // snap through it. Ramping omega prevents a burst of differential
            // at the moment of breakaway.
            float omega_delta = omega_target - g_omega_prev;
            float omega_scaled = g_omega_prev
                + constrain(omega_delta, -OMEGA_RAMP_RATE, OMEGA_RAMP_RATE);
            g_omega_prev = omega_scaled;

            // how much differential we can apply without either wheel dropping below the floor
            float max_diff = fmaxf(v_forward - MIN_WHEEL_VEL, 0.0f);
            float diff = constrain(omega_scaled * STEER_MIX_SCALE, -max_diff, max_diff);

            // --- Mix: v_forward ± differential, no reverse on either wheel ---
            float v_left  = constrain(v_forward + diff, MIN_WHEEL_VEL, MAX_VELOCITY_RPS);
            float v_right = constrain(v_forward - diff, MIN_WHEEL_VEL, MAX_VELOCITY_RPS);

            g_current_vel_right = applyAsymmetricRamp(v_right, g_current_vel_right);
            g_current_vel_left  = applyAsymmetricRamp(v_left,  g_current_vel_left);

            // --- Output ---
#if AUTO_DRY_RUN
            AUTO_SERIAL.print("DRY | dist=");   AUTO_SERIAL.print(dist, 2);
            AUTO_SERIAL.print("m | brg=");
            AUTO_SERIAL.print(g_bearing_filtered * 180.0f / PI, 1);
            if (!g_uwb_data.bearing_valid) AUTO_SERIAL.print("(1A)");
            AUTO_SERIAL.print("deg | auth=");   AUTO_SERIAL.print(authority, 2);
            AUTO_SERIAL.print(" | vfwd=");      AUTO_SERIAL.print(v_forward, 2);
            AUTO_SERIAL.print(" | omega=");     AUTO_SERIAL.print(omega_scaled, 3);
            AUTO_SERIAL.print(" | vR=");        AUTO_SERIAL.print(-g_current_vel_right, 2);
            AUTO_SERIAL.print(" | vL=");        AUTO_SERIAL.print(g_current_vel_left, 2);
            AUTO_SERIAL.print(" | dErr=");      AUTO_SERIAL.print(distController.getLastError(), 2);
            AUTO_SERIAL.print(" | bErr=");      AUTO_SERIAL.print(bearingController.getLastError() * 180.0f / PI, 1);
            AUTO_SERIAL.print("deg | bD=");     AUTO_SERIAL.println(bearingController.getLastDerivative(), 3);
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