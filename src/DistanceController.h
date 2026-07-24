#pragma once
#include <Arduino.h>

//================================================================================
// DistanceController
//
// Outer PID loop: takes a distance measurement (real UWB later, synthetic
// for now) and produces a target velocity command. This output is meant to
// feed directly into the SAME velocity ramp system already used by
// applyAsymmetricRamp() in the motor driver -- this module does NOT replace
// that ramp, it sits upstream of it.
//
// STAGE 1 USAGE (no UWB hardware):
//   Set SIM_MODE true. Instead of reading a real sensor, the controller
//   generates a synthetic distance signal (step or sine, selectable) and
//   runs it through the real PID + anti-windup + deadband logic. Log the
//   results over serial and plot them, exactly like the ODrive step tests.
//================================================================================

// --- Synthetic Test Mode ---
#define SIM_MODE true   // true = generate fake distance data, false = use real sensor input (future)

enum SimProfile { SIM_STEP, SIM_SINE, SIM_WALK };
const SimProfile SIM_PROFILE_SELECT = SIM_WALK; // change to SIM_STEP or SIM_SINE to test other input shapes

// --- Distance Loop Tuning Parameters ---
// Start conservative. These are intentionally not "tuned" yet -- Stage 1's
// whole point is to find reasonable values against a known, repeatable input.
struct DistanceControllerConfig {
    float desired_distance      = 1.5f;   // meters, target follow distance
    float kp                    = 3.0f;   // (turns/s) per meter of error
    float ki                    = 0.2f;   // integral gain
    float deadband_m            = 0.15f;  // +/- meters around setpoint treated as zero error
    float integral_clamp        = 1.0f;   // anti-windup clamp on the integral term itself
    float max_output_vel        = 3.0f;   // MUST match MAX_VELOCITY_RPS in main firmware
    float loop_dt_s             = 0.02f;  // seconds, MUST match COMMAND_INTERVAL_MS / 1000
};

class DistanceController {
public:
    DistanceController(const DistanceControllerConfig& cfg);

    // Call once per control loop cycle (same cadence as your COMMAND_INTERVAL_MS).
    // Returns the target velocity (turns/s) to feed into applyAsymmetricRamp().
    float update(float measured_distance_m);

    // Stage 1 only: produces a fake distance reading for a given elapsed time (s).
    float getSimulatedDistance(float elapsed_s) const;

    void reset(); // clears integrator -- call this if entering/leaving follow mode

    // Expose internal state for logging/plotting
    float getLastError() const { return last_error_; }
    float getIntegral() const { return integral_; }

private:
    DistanceControllerConfig cfg_;
    float integral_ = 0.0f;
    float last_error_ = 0.0f;
};