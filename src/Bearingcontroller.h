#pragma once
#include <Arduino.h>

/*
 * BearingController - PID with derivative
 *
 * The derivative term (kd) is the key addition over DistanceController:
 *   - Positive dE/dt (error growing) → D adds to command → turns initiate earlier
 *   - Negative dE/dt (error shrinking) → D subtracts from command → dampens overshoot
 *
 * This directly addresses the "delayed then overcorrects" pattern without
 * needing an extremely heavy LPF to hide the noise. The derivative is
 * computed on the already-filtered bearing input, and lightly filtered
 * itself to avoid amplifying residual noise.
 *
 * Sign convention:
 *   bearing > 0 → tag is to the RIGHT → positive omega → turn RIGHT
 */

struct BearingControllerConfig {
    float kp              = 1.5f;   // proportional
    float ki              = 0.05f;  // integral - keep small, bearing is noisy
    float kd              = 0.3f;   // derivative - main new addition
    float kd_filter_alpha = 0.4f;   // smooths the derivative term itself
    float deadband_rad    = 0.140f; // ±8° - above UWB noise floor
    float integral_clamp  = 0.3f;
    float max_omega       = 0.8f;
    float loop_dt_s       = 0.02f;
};

class BearingController {
public:
    BearingController(const BearingControllerConfig& cfg);

    // authority [0,1]: gates integrator and is used externally to scale omega output.
    float update(float bearing_rad, float authority, float dt_s);

    void reset();

    float getLastError()      const { return last_error_; }
    float getIntegral()       const { return integral_; }
    float getLastDerivative() const { return last_d_filtered_; }

private:
    BearingControllerConfig cfg_;
    float integral_        = 0.0f;
    float last_error_      = 0.0f;
    float last_d_filtered_ = 0.0f;
    bool  first_update_    = true;  // skip derivative on first call (no history yet)
};