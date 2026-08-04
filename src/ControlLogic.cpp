#include "ControlLogic.h"
#include "Config.h"
#include "OdriveComm.h"
#include "DistanceController.h"
#include "BearingController.h"

static unsigned long g_distance_fault_start = 0;
static unsigned long g_bearing_fault_start  = 0;

// --- DistanceController ---
static DistanceController distController = []() {
    DistanceControllerConfig cfg;
    cfg.kp = 2.0f; // reduced from 3.0 to reduce overshoot
    return DistanceController(cfg);
}();

// --- BearingController ---
static BearingController bearingController = []() {
    BearingControllerConfig cfg;
    cfg.kp              = BEARING_KP;
    cfg.ki              = BEARING_KI;
    cfg.deadband_rad    = BEARING_DEADBAND_RAD;
    cfg.integral_clamp  = BEARING_INTEGRAL_CLAMP;
    cfg.max_omega       = BEARING_MAX_OMEGA;
    return BearingController(cfg);
}();

// --- Bearing low-pass filter state ---
static float  g_bearing_filtered  = 0.0f;
static bool   g_bearing_lpf_ready = false; // false until first valid bearing seen

static bool distControllerReady   = false;
static bool bearingControllerReady = false;

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
        g_current_vel_left     = 0.0f;
        g_current_vel_right    = 0.0f;
        g_distance_fault        = false;   // add
        g_bearing_fault         = false;   // add
        g_distance_fault_start  = 0;       // add
        g_bearing_fault_start   = 0;       // add
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
            // --- Init on first entry ---
            if (!distControllerReady) {
                distController.reset();
                distControllerReady = true;
            }
            if (!bearingControllerReady) {
                bearingController.reset();
                bearingControllerReady = true;
            }

            // --- USB serial override (bench testing) ---
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
            float dist    = g_uwb_data.distance_m;
            float v_forward = distController.update(dist);

            // No-reverse clamp with integrator reset to prevent windup
            if (v_forward < 0.0f) {
                distController.reset();
                v_forward = 0.0f;
            }

            // --- Bearing low-pass filter ---
            // Only run when bearing data is valid (2-anchor trilateration OK).
            // On first valid reading after a gap, seed the filter with the
            // current value so it doesn't ramp in from a stale state.
            float bearing_raw = g_uwb_data.bearing_rad;
            if (g_uwb_data.bearing_valid) {
                if (!g_bearing_lpf_ready) {
                    g_bearing_filtered = bearing_raw; // seed on first valid reading
                    g_bearing_lpf_ready = true;
                } else {
                    g_bearing_filtered = BEARING_LPF_ALPHA * bearing_raw
                                       + (1.0f - BEARING_LPF_ALPHA) * g_bearing_filtered;
                }
            } else {
                // Single-anchor mode: no reliable bearing, decay filter toward 0
                // so there's no stale steering bias if bearing becomes valid again.
                g_bearing_filtered *= 0.9f;
                g_bearing_lpf_ready = false;
            }

            // --- Bearing PID → omega ---
            // Authority scales with current forward speed so the cart can't
            // spin or flip from a large differential at low speed.
            // Uses the average of ramped wheel velocities (what the cart is
            // actually doing, not the target v_forward) for honest authority.
            float current_speed = (g_current_vel_right + g_current_vel_left) / 2.0f;
            float authority = constrain(current_speed / STEER_FULL_AUTHORITY_VEL, 0.0f, 1.0f);

            float omega = 0.0f;
            if (g_uwb_data.bearing_valid) {
                omega = bearingController.update(g_bearing_filtered, authority);
            } else {
                bearingController.reset(); // don't let integral wind up in single-anchor mode
            }

            // --- Fault persistence tracking (for tag vibration alert) ---
            bool dist_out_of_band = (distController.getLastError() != 0.0f);
            if (dist_out_of_band) {
                if (g_distance_fault_start == 0) g_distance_fault_start = millis();
                g_distance_fault = (millis() - g_distance_fault_start) > FAULT_PERSIST_MS;
            } else {
                g_distance_fault_start = 0;
                g_distance_fault = false;
            }

            bool bearing_out_of_band = g_uwb_data.bearing_valid && (bearingController.getLastError() != 0.0f);
            if (bearing_out_of_band) {
                if (g_bearing_fault_start == 0) g_bearing_fault_start = millis();
                g_bearing_fault = (millis() - g_bearing_fault_start) > FAULT_PERSIST_MS;
            } else {
                g_bearing_fault_start = 0;
                g_bearing_fault = false;
            }

            // Scale omega by authority so the physical differential matches
            // what the plant can actually execute at this speed
            float omega_scaled = omega * authority * STEER_DIR;

            // --- Mixing: v_forward ± differential ---
            // bearing > 0 (tag right of centre) → omega > 0 → left faster → turns right
            // Both clamped to [0, MAX] - no reverse on either wheel in auto mode.
            float v_left  = constrain(v_forward + omega_scaled * STEER_MIX_SCALE,
                                      0.0f, MAX_VELOCITY_RPS);
            float v_right = constrain(v_forward - omega_scaled * STEER_MIX_SCALE,
                                      0.0f, MAX_VELOCITY_RPS);

            g_current_vel_right = applyAsymmetricRamp(v_right, g_current_vel_right);
            g_current_vel_left  = applyAsymmetricRamp(v_left,  g_current_vel_left);

            // --- Output ---
#if AUTO_DRY_RUN
            AUTO_SERIAL.print("DRY | dist=");   AUTO_SERIAL.print(dist, 2);
            AUTO_SERIAL.print("m | brg=");
            AUTO_SERIAL.print(g_bearing_filtered * 180.0f / PI, 1);
            if (!g_uwb_data.bearing_valid) AUTO_SERIAL.print("(1A)"); // 1-anchor mode
            AUTO_SERIAL.print("deg | auth=");   AUTO_SERIAL.print(authority, 2);
            AUTO_SERIAL.print(" | vfwd=");      AUTO_SERIAL.print(v_forward, 2);
            AUTO_SERIAL.print(" | omega=");     AUTO_SERIAL.print(omega_scaled, 3);
            AUTO_SERIAL.print(" | vR=");        AUTO_SERIAL.print(-g_current_vel_right, 2);
            AUTO_SERIAL.print(" | vL=");        AUTO_SERIAL.print(g_current_vel_left, 2);
            AUTO_SERIAL.print(" | dErr=");      AUTO_SERIAL.print(distController.getLastError(), 2);
            AUTO_SERIAL.print(" | bErr=");      AUTO_SERIAL.println(bearingController.getLastError() * 180.0f / PI, 1);
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