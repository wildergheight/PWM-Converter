#include "BearingController.h"

BearingController::BearingController(const BearingControllerConfig& cfg)
    : cfg_(cfg) {}

void BearingController::reset() {
    integral_   = 0.0f;
    last_error_ = 0.0f;
}

float BearingController::update(float bearing_rad, float authority) {
    float error = bearing_rad;

    bool in_deadband = (fabsf(error) < cfg_.deadband_rad);

    if (in_deadband) {
        error = 0.0f;
        integral_ *= 0.95f; // decay when settled, same pattern as DistanceController
    }

    float p_term = cfg_.kp * error;

    // Gate integrator accumulation on authority so it doesn't wind up
    // while the cart is stationary or barely moving. Without this, a
    // parked cart with a bearing error would accumulate integral and then
    // burst-turn the moment it starts moving forward.
    if (authority > 0.1f) {
        integral_ += error * cfg_.loop_dt_s * authority;
        integral_ = constrain(integral_, -cfg_.integral_clamp, cfg_.integral_clamp);
    } else {
        integral_ *= 0.95f; // decay integrator while authority is near zero
    }

    float i_term = cfg_.ki * integral_;
    float output = p_term + i_term;
    output = constrain(output, -cfg_.max_omega, cfg_.max_omega);

    last_error_ = error;
    return output;
}