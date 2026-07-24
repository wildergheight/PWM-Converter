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
    handleChannelScanning(); // Start hunting if the remote is gone
    checkAutoSerial();
    checkODriveStatus();     // Ask for and parse ODrive status (no-op if TUNING_MODE)

    if (millis() - g_last_command_time < COMMAND_INTERVAL_MS) return;
    g_last_command_time = millis();

    ControlState desired_state = determineDesiredState();
    handleStateTransition(desired_state);

    // Kept outside any TUNING_MODE guard so state tracking stays accurate
    // even when ODrive writes are disabled for bench tuning.
    g_control_state = desired_state;

    executeControlState();
}
