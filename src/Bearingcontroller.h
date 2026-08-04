#pragma once
#include <Arduino.h>

/*
 * BearingController
 *
 * Outer bearing PID loop: takes a filtered bearing measurement (radians)
 * and produces an angular rate command (omega, rad/s) to feed into the
 * differential mixing stage.
 *
 * Deliberately mirrors DistanceController's structure so tuning patterns
 * are familiar. Key differences:
 *   - Integrator is gated on steering authority (won't wind up when the
 *     cart is barely moving and can't physically turn).
 *   - Input should be low-pass filtered before calling update() — raw UWB
 *     bearing is too noisy to feed directly into a PID.
 *
 * Mixing (in ControlLogic.cpp):
 *   v_left  = v_forward + omega * STEER_MIX_SCALE
 *   v_right = v_forward - omega * STEER_MIX_SCALE
 *   (both clamped to [0, MAX_VELOCITY_RPS])
 *
 * Sign convention:
 *   bearing > 0 → tag is to the RIGHT (toward Anchor B)
 *   omega   > 0 → turn RIGHT (left wheel faster)
 */

struct BearingControllerConfig {
    float kp             = 1.0f;    // (rad/s) per radian of bearing error
    float ki             = 0.05f;   // integral gain - start very small
    float deadband_rad   = 0.087f;  // ±5° - below this, no steering command
    float integral_clamp = 0.3f;    // tighter than distance - bearing is noisier
    float max_omega      = 0.5f;    // rad/s - caps how aggressively it can turn
    float loop_dt_s      = 0.02f;
};

class BearingController {
public:
    BearingController(const BearingControllerConfig& cfg);

    // authority [0,1]: scales integrator accumulation and is used externally
    // to scale the omega output. Pass current forward speed / STEER_FULL_AUTHORITY_VEL.
    float update(float bearing_rad, float authority);

    void reset();

    float getLastError()  const { return last_error_; }
    float getIntegral()   const { return integral_; }

private:
    BearingControllerConfig cfg_;
    float integral_   = 0.0f;
    float last_error_ = 0.0f;
};