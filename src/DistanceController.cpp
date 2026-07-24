#include "DistanceController.h"

DistanceController::DistanceController(const DistanceControllerConfig& cfg)
    : cfg_(cfg) {}

void DistanceController::reset() {
    integral_ = 0.0f;
    last_error_ = 0.0f;
}

namespace {
    // Integral of a half-cosine-eased velocity ramp from v0 to v1 over
    // duration D, evaluated from 0 to T (T must be <= D). Used to get an
    // exact closed-form distance traveled during acceleration/deceleration,
    // rather than approximating the area under the curve.
    float easedRampDistance(float v0, float v1, float D, float T) {
        if (T <= 0.0f) return 0.0f;
        if (T > D) T = D;
        float dv = v1 - v0;
        return v0 * T + dv * 0.5f * (T - (D / PI) * sinf(PI * T / D));
    }
}

float DistanceController::getSimulatedDistance(float elapsed_s) const {
    if (SIM_PROFILE_SELECT == SIM_STEP) {
        // STEP profile: mimics someone standing still, then suddenly walking away,
        // then stopping again -- similar shape to your ODrive step tests.
        if (elapsed_s < 3.0f) {
            return 1.5f;
        } else if (elapsed_s < 6.0f) {
            return 3.0f;
        } else {
            return 1.5f;
        }
    } else if (SIM_PROFILE_SELECT == SIM_SINE) {
        // SINE profile: mimics an operator drifting back and forth in distance.
        const float amplitude = 0.5f;
        const float period_s = 4.0f;
        return cfg_.desired_distance + amplitude * sinf(2.0f * PI * elapsed_s / period_s);
    } else {
        // WALK profile: mimics a realistic round-trip -- operator walks
        // away, pauses, then walks back to the setpoint. Smooth
        // acceleration/deceleration throughout (no instant jumps), which
        // is much closer to real walking dynamics than a hard step or a
        // continuous oscillation. This is the best test yet for "does
        // this feel twitchy or well-matched" across a realistic full
        // interaction: walking away, holding distance, and the cart
        // catching back up as the operator returns.
        //
        // Timeline:
        //   0.0 - 1.0s : standing still at the setpoint
        //   1.0 - 2.0s : smoothly accelerates away
        //   2.0 - 3.5s : holds outbound pace (steady-state tracking, outbound)
        //   3.5 - 4.5s : smoothly decelerates to a stop, far from setpoint
        //   4.5 - 5.5s : pauses at the far distance
        //   5.5 - 6.5s : smoothly accelerates back toward the setpoint
        //   6.5 - 8.0s : holds return pace (steady-state tracking, inbound)
        //   8.0 - 9.0s : smoothly decelerates back to a stop AT the setpoint
        //   9.0s+      : standing still again, cart should be holding steady
        const float speed = 1.3f; // m/s, typical brisk walking pace

        const float t_accel_out_start = 1.0f, accel_dur = 1.0f;
        const float t_hold_out_start  = t_accel_out_start + accel_dur;        // 2.0
        const float hold_dur          = 1.5f;
        const float t_decel_out_start = t_hold_out_start + hold_dur;          // 3.5
        const float decel_dur         = 1.0f;
        const float t_pause_start     = t_decel_out_start + decel_dur;        // 4.5
        const float pause_dur         = 1.0f;
        const float t_accel_in_start  = t_pause_start + pause_dur;            // 5.5
        const float t_hold_in_start   = t_accel_in_start + accel_dur;        // 6.5
        const float t_decel_in_start  = t_hold_in_start + hold_dur;          // 8.0
        // settle starts at t_decel_in_start + decel_dur = 9.0

        float distance = cfg_.desired_distance;

        // Outbound acceleration
        distance += easedRampDistance(0.0f, speed, accel_dur, elapsed_s - t_accel_out_start);

        // Outbound hold (constant speed away)
        if (elapsed_s > t_hold_out_start) {
            float t_in_hold = fminf(elapsed_s, t_decel_out_start) - t_hold_out_start;
            distance += speed * fmaxf(t_in_hold, 0.0f);
        }

        // Outbound deceleration (to a stop, far from setpoint)
        if (elapsed_s > t_decel_out_start) {
            distance += easedRampDistance(speed, 0.0f, decel_dur, elapsed_s - t_decel_out_start);
        }

        // Pause phase: no additional distance change (handled implicitly --
        // nothing added here, distance just holds at whatever it reached)

        // Inbound acceleration (negative direction, walking back)
        if (elapsed_s > t_accel_in_start) {
            distance += easedRampDistance(0.0f, -speed, accel_dur, elapsed_s - t_accel_in_start);
        }

        // Inbound hold (constant speed back toward setpoint)
        if (elapsed_s > t_hold_in_start) {
            float t_in_hold = fminf(elapsed_s, t_decel_in_start) - t_hold_in_start;
            distance += -speed * fmaxf(t_in_hold, 0.0f);
        }

        // Inbound deceleration (smoothly stopping back at the setpoint)
        if (elapsed_s > t_decel_in_start) {
            distance += easedRampDistance(-speed, 0.0f, decel_dur, elapsed_s - t_decel_in_start);
        }

        return distance;
    }
}

float DistanceController::update(float measured_distance_m) {
    float error = measured_distance_m - cfg_.desired_distance;

    bool in_deadband = (fabsf(error) < cfg_.deadband_m);

    // --- Deadband ---
    // Treat small errors as zero output contribution from P, but actively
    // decay the integral while inside the deadband so it doesn't stay
    // pinned at whatever value it last wound up to.
    if (in_deadband) {
        error = 0.0f;
        integral_ *= 0.95f; // decay toward zero each cycle while settled
    }

    // --- Proportional term ---
    float p_term = cfg_.kp * error;

    // --- Integral term with anti-windup ---
    integral_ += error * cfg_.loop_dt_s;
    integral_ = constrain(integral_, -cfg_.integral_clamp, cfg_.integral_clamp);
    float i_term = cfg_.ki * integral_;

    float output = p_term + i_term;
    output = constrain(output, -cfg_.max_output_vel, cfg_.max_output_vel);

    last_error_ = error;
    return output;
}