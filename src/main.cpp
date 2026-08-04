/*
 * ESP32 ESP-NOW/Serial to ODrive Serial (ASCII) Converter
 *
 * Version: 10.0 (modularized)
 *
 * Description:
 * This code receives control commands from a remote (e.g., a T-Beam) via
 * ESP-NOW and translates them into ODrive torque commands. It retains the
 * ability to be overridden by velocity commands from a host computer via USB.
 *
 * Control Priority & Mode:
 * 1. ESP-NOW Failsafe (Highest): Velocity Control (0 vel)
 * 2. Low Voltage Cutoff: Velocity Control (0 vel), latched
 * 3. Automation Serial Command (USB): Velocity Control
 * 4. ESP-NOW Control (Lowest): Torque Control
 *
 * See src/ for the implementation, split by concern:
 *   Config.h        - pins, tuning constants, TUNING_MODE
 *   Globals.h/.cpp   - shared state (ControlData, enums, extern globals)
 *   OdriveComm.h/.cpp - ODrive serial status polling + command output
 *   EspNowHandler.h/.cpp - ESP-NOW init, recv callback, channel hunting
 *   AutoSerial.h/.cpp - USB command parsing
 *   ControlLogic.h/.cpp - state machine: decide / transition / execute
 */

#include <Arduino.h>
#include "Config.h"
#include "Globals.h"
#include "OdriveComm.h"
#include "EspNowHandler.h"
#include "AutoSerial.h"
#include "ControlLogic.h"

void setup() {
    AUTO_SERIAL.begin(AUTO_BAUD_RATE);
    ODRIVE_SERIAL.begin(ODRIVE_BAUD_RATE, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);

    AUTO_SERIAL.println("ESP32 ODrive Bridge v10.0 (modularized) Ready.");

    pinMode(RIGHT_LED, OUTPUT);
    pinMode(LEFT_LED, OUTPUT);
    pinMode(BATT_LED, OUTPUT);

    digitalWrite(RIGHT_LED, HIGH);
    digitalWrite(LEFT_LED, HIGH);
    digitalWrite(BATT_LED, HIGH);

    initEspNow();

    // Cart is ready
    digitalWrite(RIGHT_LED, LOW);
    digitalWrite(LEFT_LED, LOW);
    digitalWrite(BATT_LED, LOW);
}

void loop() {
    // handleChannelScanning(); // Start hunting if the remote is gone
    checkAutoSerial();
    checkODriveStatus();     // Ask for and parse ODrive status (no-op if TUNING_MODE)

    if (millis() - g_last_status_broadcast_time > STATUS_BROADCAST_INTERVAL_MS) {
        g_last_status_broadcast_time = millis();
        sendStatusAlert();
    }
    
    if (millis() - g_last_command_time < COMMAND_INTERVAL_MS) return;
    g_last_command_time = millis();

    ControlState desired_state = determineDesiredState();
    handleStateTransition(desired_state);

    // Kept outside any TUNING_MODE guard so state tracking stays accurate
    // even when ODrive writes are disabled for bench tuning.
    g_control_state = desired_state;

    executeControlState();
}


// /*
//  * Wheel Trim Calibration - Standalone Test
//  * Runs both ODrive axes at equal commanded velocity with adjustable trim.
//  * Flash this temporarily, calibrate, then port TRIM_LEFT/TRIM_RIGHT
//  * values into main.cpp and reflash that.
//  *
//  * Serial commands (115200 baud):
//  *   g          - go (start motors)
//  *   s          - stop (brake)
//  *   +  / -     - nudge RIGHT trim up/down by 0.01
//  *   [  / ]     - nudge LEFT trim up/down by 0.01
//  *   v0.5       - set base test velocity (turns/sec), e.g. "v0.5"
//  *   p          - print current trim + velocity values
//  */

// /*
//  * Wheel Trim Calibration - Standalone Test (with dead-man's-switch failsafe)
//  *
//  * Serial commands (115200 baud):
//  *   HOLD 'g'   - run motors (must keep arriving repeatedly - hold the key down)
//  *   release/no data for HEARTBEAT_TIMEOUT_MS - auto-stop
//  *   +  / -     - nudge RIGHT trim up/down by 0.01
//  *   [  / ]     - nudge LEFT trim up/down by 0.01
//  *   v0.5       - set base test velocity (turns/sec), e.g. "v0.5"
//  *   p          - print current trim + velocity values
//  */

// #include <Arduino.h>

// #define ODRIVE_SERIAL Serial2
// const int ODRIVE_RX_PIN = 16;
// const int ODRIVE_TX_PIN = 17;
// const long ODRIVE_BAUD_RATE = 115200;

// float TRIM_RIGHT = 1.00;
// float TRIM_LEFT  = 1.00;
// const float DIR_RIGHT = -1.0;  // physically reversed
// const float DIR_LEFT  =  1.0;
// float BASE_VEL   = 0.8;   // turns/sec

// const unsigned long CMD_INTERVAL_MS = 20;
// unsigned long g_last_cmd_time = 0;

// // --- Dead-man's-switch failsafe ---
// const unsigned long HEARTBEAT_TIMEOUT_MS = 250; // must see 'g' at least this often
// unsigned long g_last_heartbeat_time = 0;

// const float RAMP_RATE = 1.5; // turns/sec^2 - tune to taste, lower = smoother

// float g_current_right = 0.0;
// float g_current_left  = 0.0;

// float rampTowards(float target, float current) {
//     float max_step = RAMP_RATE * (CMD_INTERVAL_MS / 1000.0);
//     float delta = target - current;
//     return current + constrain(delta, -max_step, max_step);
// }

// void setODriveVelocityMode() {
//     ODRIVE_SERIAL.println("w axis0.controller.config.control_mode 2");
//     ODRIVE_SERIAL.println("w axis1.controller.config.control_mode 2");
//     ODRIVE_SERIAL.println("w axis0.controller.config.input_mode 1");
//     ODRIVE_SERIAL.println("w axis1.controller.config.input_mode 1");
// }

// void printStatus() {
//     Serial.println("---");
//     Serial.println("Base Vel: " + String(BASE_VEL, 3));
//     Serial.println("TRIM_RIGHT: " + String(TRIM_RIGHT, 3));
//     Serial.println("TRIM_LEFT:  " + String(TRIM_LEFT, 3));
//     Serial.println("---");
// }

// void handleSerial() {
//     while (Serial.available()) {
//         char c = Serial.read();
//         switch (c) {
//             case 'g':
//                 g_last_heartbeat_time = millis(); // heartbeat received - keep running
//                 break;
//             case '+': TRIM_RIGHT += 0.01; printStatus(); break;
//             case '-': TRIM_RIGHT -= 0.01; printStatus(); break;
//             case '[': TRIM_LEFT  -= 0.01; printStatus(); break;
//             case ']': TRIM_LEFT  += 0.01; printStatus(); break;
//             case 'p': printStatus(); break;
//             case 'v': {
//                 String val = Serial.readStringUntil('\n');
//                 BASE_VEL = val.toFloat();
//                 Serial.println("Base Vel set to " + String(BASE_VEL, 3));
//                 break;
//             }
//         }
//     }
// }

// void setup() {
//     Serial.begin(115200);
//     ODRIVE_SERIAL.begin(ODRIVE_BAUD_RATE, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);
//     delay(500);
//     setODriveVelocityMode();
//     Serial.println("Trim Test Ready. HOLD 'g' to run. Release/unplug to stop.");
//     Serial.println("'+/-'=right trim  '[/]'=left trim  'p'=print  'vX.X'=set speed");
//     printStatus();
// }

// void loop() {
//     handleSerial();

//     if (millis() - g_last_cmd_time < CMD_INTERVAL_MS) return;
//     g_last_cmd_time = millis();

//     bool alive = (millis() - g_last_heartbeat_time) < HEARTBEAT_TIMEOUT_MS;

//     if (alive) {
//         float right_cmd = BASE_VEL * TRIM_RIGHT * DIR_RIGHT;
//         float left_cmd  = BASE_VEL * TRIM_LEFT * DIR_LEFT;
//         g_current_right = rampTowards(right_cmd, g_current_right);
//         g_current_left = rampTowards(left_cmd, g_current_left);
//         ODRIVE_SERIAL.println("w axis0.controller.input_vel " + String(g_current_right, 3));
//         ODRIVE_SERIAL.println("w axis1.controller.input_vel " + String(g_current_left, 3));
//     } else {
//         ODRIVE_SERIAL.println("w axis0.controller.input_vel 0");
//         ODRIVE_SERIAL.println("w axis1.controller.input_vel 0");
//     }
// }