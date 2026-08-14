#include "OdriveComm.h"
#include "Config.h"

// --- Wheel Trim Calibration ---
// Multiplicative correction for mechanical asymmetry (wheel diameter, motor Kv, etc.)
// >1.0 = wheel runs faster/harder relative to its pair, <1.0 = weaker
const float TRIM_RIGHT = 1.00;  // start here, adjust during calibration
const float TRIM_LEFT  = 1.00;

void checkODriveStatus() {
#if TUNING_MODE
    return; // odrivetool / Python scripts have exclusive access to the serial line.
#else
    // --- Send the next query ---
    if (millis() - g_last_query_time > QUERY_INTERVAL_MS) {
        g_last_query_time = millis();

        switch (g_pending_query) {
            case QUERY_VOLTAGE:
                ODRIVE_SERIAL.println("r vbus_voltage");
                g_awaiting_reply = QUERY_VOLTAGE;
                g_pending_query  = QUERY_AXIS0_ERROR;
                break;

            case QUERY_AXIS0_ERROR:
                ODRIVE_SERIAL.println("r axis0.error");
                g_awaiting_reply = QUERY_AXIS0_ERROR;
                g_pending_query  = QUERY_AXIS1_ERROR;
                break;

            case QUERY_AXIS1_ERROR:
                ODRIVE_SERIAL.println("r axis1.error");
                g_awaiting_reply = QUERY_AXIS1_ERROR;
                g_pending_query  = QUERY_VOLTAGE;
                break;
        }
    }

    // --- Parse any available ODrive response ---
    while (ODRIVE_SERIAL.available()) {
        String response = ODRIVE_SERIAL.readStringUntil('\n');
        response.trim();
        if (response.length() == 0) continue;

        switch (g_awaiting_reply) {
            case QUERY_VOLTAGE: {
                float parsed_voltage = response.toFloat();
                if (parsed_voltage > 10.0) {
                    g_bus_voltage = parsed_voltage;
                    // AUTO_SERIAL.println("VBUS: " + String(g_bus_voltage, 1) + "V");

                    if (ENABLE_LOW_VOLTAGE_CUTOFF && g_bus_voltage < LOW_VOLTAGE_CUTOFF) {
                        g_lvc_consecutive_count++;
                        AUTO_SERIAL.println("LVC Warning: " + String(g_lvc_consecutive_count) + "/" + String(LVC_CONSECUTIVE_READINGS));
                    } else {
                        g_lvc_consecutive_count = 0; // Reset on any healthy reading
                    }
                }
                break;
            }

            case QUERY_AXIS0_ERROR: {
                int error_code = strtol(response.c_str(), nullptr, 0); // handles 0x... and decimal
                g_axis0_fault = (error_code != 0);
                digitalWrite(RIGHT_LED, g_axis0_fault ? HIGH : LOW);
                if (g_axis0_fault) {
                    AUTO_SERIAL.println("FAULT axis0: 0x" + String(error_code, HEX));
                }
                break;
            }

            case QUERY_AXIS1_ERROR: {
                int error_code = strtol(response.c_str(), nullptr, 0);
                g_axis1_fault = (error_code != 0);
                digitalWrite(LEFT_LED, g_axis1_fault ? HIGH : LOW);
                if (g_axis1_fault) {
                    AUTO_SERIAL.println("FAULT axis1: 0x" + String(error_code, HEX));
                }
                break;
            }
        }
    }
#endif
}

void sendOdriveModeTransition(ControlState from_state, ControlState to_state) {
#if TUNING_MODE
    (void)from_state; (void)to_state;
    return;
#else
    bool to_velocity = (to_state == STATE_BRAKING || to_state == STATE_VELOCITY_AUTO || to_state == STATE_LOW_VOLTAGE_CUTOFF);
    bool to_torque = (to_state == STATE_TORQUE_ESPNOW);

    if (to_velocity && from_state == STATE_TORQUE_ESPNOW) {
        AUTO_SERIAL.println("STATE: Switching ODrive to VELOCITY_CONTROL");
        ODRIVE_SERIAL.println("w axis0.controller.config.control_mode 2");
        ODRIVE_SERIAL.println("w axis1.controller.config.control_mode 2");

        // Input mode: VEL_RAMP-capable passthrough
        ODRIVE_SERIAL.println("w axis0.controller.config.input_mode 1");
        ODRIVE_SERIAL.println("w axis1.controller.config.input_mode 1");

    } else if (to_torque && from_state != STATE_TORQUE_ESPNOW) {
        AUTO_SERIAL.println("STATE: Switching ODrive to TORQUE_CONTROL");
        ODRIVE_SERIAL.println("w axis0.controller.config.control_mode 1");
        ODRIVE_SERIAL.println("w axis1.controller.config.control_mode 1");

        // Input mode: PASSTHROUGH
        ODRIVE_SERIAL.println("w axis0.controller.config.input_mode 1");
        ODRIVE_SERIAL.println("w axis1.controller.config.input_mode 1");

        // NEW: input_torque isn't cleared by switching control_mode -- it
        // still holds whatever was last sent before this axis LEFT torque
        // mode (possibly mid-turn, from before auto mode was even engaged).
        // Zero it explicitly so the motor doesn't act on stale torque the
        // instant torque mode takes effect, before our first real command
        // this cycle.
        ODRIVE_SERIAL.println("c 0 0");
        ODRIVE_SERIAL.println("c 1 0");
    }
#endif
}

void sendOdriveBrake() {
#if TUNING_MODE
    return;
#else
    ODRIVE_SERIAL.println("v 0 0");
    ODRIVE_SERIAL.println("v 1 0");
#endif
}

void sendOdriveVelocity(float right_vel, float left_vel) {
#if TUNING_MODE
    (void)right_vel; (void)left_vel;
    return;
#else
    ODRIVE_SERIAL.println("w axis0.controller.input_vel " + String(right_vel * TRIM_RIGHT, 2));
    ODRIVE_SERIAL.println("w axis1.controller.input_vel " + String(left_vel * TRIM_LEFT, 2));
#endif
}

void sendOdriveTorque(float right_torque, float left_torque) {
#if TUNING_MODE
    (void)right_torque; (void)left_torque;
    return;
#else
    ODRIVE_SERIAL.print("c 0 " + String(right_torque * TRIM_RIGHT, 2) + "\n");
    ODRIVE_SERIAL.print("c 1 " + String(left_torque * TRIM_LEFT, 2) + "\n");
#endif
}
