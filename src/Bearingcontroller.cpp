#include "Bearingcontroller.h"

BearingController::BearingController(const BearingControllerConfig& cfg)
    : cfg_(cfg) {}

void BearingController::reset() {
    integral_        = 0.0f;
    last_error_      = 0.0f;
    last_d_filtered_ = 0.0f;
    first_update_    = true;
}

float BearingController::update(float bearing_rad, float authority, float dt_s) {
    float error = bearing_rad;
    bool in_deadband = (fabsf(error) < cfg_.deadband_rad);

    if (in_deadband) {
        error = 0.0f;
        integral_        *= 0.95f;
        last_d_filtered_ *= 0.95f; // also decay derivative when settled
    }

    // --- Proportional ---
    float p_term = cfg_.kp * error;

    // --- Integral (gated on authority to prevent windup while stationary) ---
    if (authority > 0.1f) {
        integral_ += error * dt_s * authority;
        integral_  = constrain(integral_, -cfg_.integral_clamp, cfg_.integral_clamp);
    } else {
        integral_ *= 0.95f;
    }
    float i_term = cfg_.ki * integral_;

    // --- Derivative ---
    // Computed from the filtered bearing input so it reflects the trend of
    // the smoothed signal rather than amplifying raw UWB noise.
    // Skip on first update (no previous error to diff against).
    float d_term = 0.0f;
    if (!first_update_) {
        float d_raw = (error - last_error_) / dt_s;

        // Lightly filter the derivative itself to smooth residual noise
        last_d_filtered_ = cfg_.kd_filter_alpha * d_raw
                         + (1.0f - cfg_.kd_filter_alpha) * last_d_filtered_;
        d_term = cfg_.kd * last_d_filtered_;
    }
    first_update_ = false;
    last_error_   = error;

    float output = p_term + i_term + d_term;
    output = constrain(output, -cfg_.max_omega, cfg_.max_omega);
    return output;
}