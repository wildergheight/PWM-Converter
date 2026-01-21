/*
 * ESP32 ESP-NOW/Serial to ODrive Serial (ASCII) Converter
 * * Author: Gemini
 * * Date: August 25, 2025
 * * Version: 9.0 (ESP-NOW Control)
 * *
 * * Description:
 * This code receives control commands from a remote (e.g., a T-Beam) via
 * ESP-NOW and translates them into ODrive torque commands. It retains the
 * ability to be overridden by velocity commands from a host computer via USB.
 * 
 * Control Priority & Mode:
 * 1. ESP-NOW Failsafe (Highest): Velocity Control (0 vel)
 * 2. Automation Serial Command (USB): Velocity Control
 * 3. ESP-NOW Control (Lowest): Torque Control
 */

#include <Arduino.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

//================================================================================
// Configuration
//================================================================================

// --- ODrive Serial Port Configuration ---
#define ODRIVE_SERIAL Serial2
const int ODRIVE_RX_PIN = 16;
const int ODRIVE_TX_PIN = 17;
const long ODRIVE_BAUD_RATE = 115200;

// --- Automation Serial Port (Main USB) ---
#define AUTO_SERIAL Serial
const long AUTO_BAUD_RATE = 115200;

// --- Tuning Parameters ---
const float MAX_TORQUE = 30.0;
const float STEERING_SENSITIVITY = 0.5;
const float THROTTLE_EXPO = 2.3;

const float MAX_VELOCITY_RPS = 4.0;
// --- Velocity Tuning ---
const float VEL_RAMP_RATE = 2.0; // Acceleration in turns/s^2 (lower = smoother)

// --- Battery & Low Voltage Cutoff ---
const bool ENABLE_LOW_VOLTAGE_CUTOFF = true;
// IMPORTANT: Set this to a SAFE voltage for your battery pack.
// For a 36V 10S Li-ion pack, 32V (3.2V/cell) is a safe cutoff point.
const float LOW_VOLTAGE_CUTOFF = 32.0; 
const unsigned long VOLTAGE_CHECK_INTERVAL_MS = 2000; // Check voltage every 2 seconds

// --- Failsafe and Timing ---
const unsigned long COMMAND_INTERVAL_MS = 20;
const unsigned long ESPNOW_FAILSAFE_TIMEOUT_MS = 500; // Fallback to brake after 500ms
const unsigned long AUTO_FAILSAFE_TIMEOUT_MS = 500;   // Fallback to ESP-NOW after 500ms

// --- Torque Ramp Rate ---
const float TORQUE_RAMP_RATE = 2.0;  // [Nm/sec]
const float MAX_TORQUE_CHANGE_PER_CYCLE = TORQUE_RAMP_RATE * (COMMAND_INTERVAL_MS / 1000.0);

//================================================================================
// Global Variables
//================================================================================

// --- State Machine for Control ---
enum ControlState { STATE_TORQUE_ESPNOW, STATE_VELOCITY_AUTO, STATE_BRAKING, STATE_LOW_VOLTAGE_CUTOFF };
ControlState g_control_state = STATE_BRAKING; // Start in BRAKING state for safety

// --- ESP-NOW Data ---
typedef struct ControlData {
    float throttle;
    float steering;
    bool button_state;  // ADDED: 0 for off, 1 for on
    bool auto_mode;
} ControlData;

ControlData espnowData;
volatile unsigned long g_last_espnow_command_time = 0;

// --- Automation Command Variables ---
String g_auto_serial_buffer = "";
float g_auto_right_norm = 0.0;
float g_auto_left_norm = 0.0;
unsigned long g_last_auto_command_time = 0;

// --- Rate Limiter State (for Torque mode only) ---
float g_last_sent_right_torque = 0.0;
float g_last_sent_left_torque = 0.0;

// --- Voltage & LVC Variables ---
float g_bus_voltage = 0.0; // Stores the last read voltage from ODrive
unsigned long g_last_voltage_check_time = 0;
bool g_lvc_activated = false; // Latches the LVC state once triggered


unsigned long g_last_command_time = 0;

//================================================================================
// ESP-NOW Callback
//================================================================================
void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&espnowData, incomingData, sizeof(espnowData));
    g_last_espnow_command_time = millis();
}

//================================================================================
// Helper & Automation Parsing Functions
//================================================================================
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void checkODriveVoltage() {
    // Periodically ask the ODrive for its voltage
    if (ENABLE_LOW_VOLTAGE_CUTOFF && millis() - g_last_voltage_check_time > VOLTAGE_CHECK_INTERVAL_MS) {
        ODRIVE_SERIAL.println("r vbus_voltage");
        g_last_voltage_check_time = millis();
    }

    // Check if the ODrive has sent a reply
    while (ODRIVE_SERIAL.available()) {
        String response = ODRIVE_SERIAL.readStringUntil('\n');
        float parsed_voltage = response.toFloat();
        // Basic validation to filter out noise or parsing errors
        if (parsed_voltage > 10.0) { 
            g_bus_voltage = parsed_voltage;
            AUTO_SERIAL.println("VBUS: " + String(g_bus_voltage) + "V");
        }
    }
}

void parseAutoCommand(String cmd) {
    cmd.trim();
    int r_idx = cmd.indexOf('R');
    int l_idx = cmd.indexOf('L');
    int comma_idx = cmd.indexOf(',');

    if (r_idx != -1 && l_idx != -1 && comma_idx != -1) {
        String r_val_str = cmd.substring(r_idx + 2, comma_idx);
        String l_val_str = cmd.substring(l_idx + 2);
        
        g_auto_right_norm = constrain(r_val_str.toFloat(), -1.0, 1.0);
        g_auto_left_norm = constrain(l_val_str.toFloat(), -1.0, 1.0);
        g_last_auto_command_time = millis();
    }
}

void checkAutoSerial() {
    while (AUTO_SERIAL.available()) {
        char c = AUTO_SERIAL.read();
        if (c == '\n') {
            parseAutoCommand(g_auto_serial_buffer);
            g_auto_serial_buffer = "";
        } else {
            g_auto_serial_buffer += c;
        }
    }
}

//================================================================================
// Main Program
//================================================================================
void setup() {
    AUTO_SERIAL.begin(AUTO_BAUD_RATE);
    ODRIVE_SERIAL.begin(ODRIVE_BAUD_RATE, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);
    
    AUTO_SERIAL.println("ESP32 ODrive Bridge v9.0 (ESP-NOW) Ready.");
    
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    AUTO_SERIAL.print("My MAC Address: ");
    AUTO_SERIAL.println(WiFi.macAddress());

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        AUTO_SERIAL.println("Error initializing ESP-NOW");
        return;
    }
    
    // Register the receive callback
    esp_now_register_recv_cb(onDataRecv);
}

void loop() {
    checkAutoSerial();
    checkODriveVoltage(); // Ask for and parse ODrive voltage

    if (millis() - g_last_command_time < COMMAND_INTERVAL_MS) return;
    g_last_command_time = millis();

    // --- DETERMINE CONTROL STATE ---
    ControlState desired_state;
    
    bool espnow_failsafe = (millis() - g_last_espnow_command_time > ESPNOW_FAILSAFE_TIMEOUT_MS);
    bool auto_override = (millis() - g_last_auto_command_time < AUTO_FAILSAFE_TIMEOUT_MS);

    if (espnow_failsafe || espnowData.button_state) {
        desired_state = STATE_BRAKING;
    } else if (ENABLE_LOW_VOLTAGE_CUTOFF && g_bus_voltage < LOW_VOLTAGE_CUTOFF && g_bus_voltage > 0.0) {
        desired_state = STATE_LOW_VOLTAGE_CUTOFF;
    } else if (auto_override || espnowData.auto_mode) {
        desired_state = STATE_VELOCITY_AUTO;
    } else {
        desired_state = STATE_TORQUE_ESPNOW;
    }

    // --- HANDLE ODRIVE MODE TRANSITIONS ---
    if (desired_state != g_control_state) {
        if ((desired_state == STATE_BRAKING || desired_state == STATE_VELOCITY_AUTO || desired_state == STATE_LOW_VOLTAGE_CUTOFF) && g_control_state == STATE_TORQUE_ESPNOW) {
            AUTO_SERIAL.println("STATE: Switching ODrive to VELOCITY_CONTROL");
            ODRIVE_SERIAL.println("w axis0.controller.config.control_mode 2");
            ODRIVE_SERIAL.println("w axis1.controller.config.control_mode 2");

                // NEW: Set Input Mode to VEL_RAMP (2)
            ODRIVE_SERIAL.println("w axis0.controller.config.input_mode 2");
            ODRIVE_SERIAL.println("w axis1.controller.config.input_mode 2");

            // NEW: Set the Acceleration (Ramp Rate)
            ODRIVE_SERIAL.println("w axis0.controller.config.vel_ramp_rate " + String(VEL_RAMP_RATE));
            ODRIVE_SERIAL.println("w axis1.controller.config.vel_ramp_rate " + String(VEL_RAMP_RATE));
            
        } else if (desired_state == STATE_TORQUE_ESPNOW && g_control_state != STATE_TORQUE_ESPNOW) {
            AUTO_SERIAL.println("STATE: Switching ODrive to TORQUE_CONTROL");
            ODRIVE_SERIAL.println("w axis0.controller.config.control_mode 1");
            ODRIVE_SERIAL.println("w axis1.controller.config.control_mode 1");

                // NEW: Set Input Mode to PASSTHROUGH (1)
            ODRIVE_SERIAL.println("w axis0.controller.config.input_mode 1");
            ODRIVE_SERIAL.println("w axis1.controller.config.input_mode 1");
        }

        // When LVC is triggered for the first time
        if(desired_state == STATE_LOW_VOLTAGE_CUTOFF && !g_lvc_activated){
            g_lvc_activated = true; // Latch the LVC state
            AUTO_SERIAL.println("!!! LOW VOLTAGE CUTOFF ACTIVATED !!!");
            AUTO_SERIAL.println("Voltage: " + String(g_bus_voltage) + "V. System halted.");
        }

        g_control_state = desired_state;
        delay(5);
    }

    // --- EXECUTE ACTION BASED ON STATE ---
    switch(g_control_state) {
        case STATE_LOW_VOLTAGE_CUTOFF:
            // This is the highest priority state. Brakes are applied and held, unless system is reset.
            ODRIVE_SERIAL.println("v 0 0");
            ODRIVE_SERIAL.println("v 1 0");
            if (g_lvc_activated) {
                while(1) { // Halt system
                    ODRIVE_SERIAL.println("v 0 0");
                    ODRIVE_SERIAL.println("v 1 0");

                    AUTO_SERIAL.print("v 0 0 ");
                    AUTO_SERIAL.println("v 1 0");
                    delay(5);
                }
            }
            break;

        case STATE_BRAKING:
            ODRIVE_SERIAL.println("v 0 0");
            ODRIVE_SERIAL.println("v 1 0");

            AUTO_SERIAL.print("v 0 0 ");
            AUTO_SERIAL.println("v 1 0");
                
            break;

        case STATE_VELOCITY_AUTO:
            {
                AUTO_SERIAL.print("Auto Right: " + String(espnowData.throttle, 2) + " ");
                AUTO_SERIAL.println("Auto Left: " + String(espnowData.steering, 2));
                if (espnowData.auto_mode){
                    g_auto_right_norm = espnowData.throttle; //THROTTLE IS RIGHT MOTOR IN ESPNOW AUTO MODE
                    g_auto_left_norm = espnowData.steering; //STEERING IS LEFT MOTOR IN ESPNOW AUTO MODE
                }
                float right_vel = g_auto_right_norm * MAX_VELOCITY_RPS;
                float left_vel = g_auto_left_norm * MAX_VELOCITY_RPS;
                ODRIVE_SERIAL.println("w axis0.controller.input_vel " + String(right_vel, 2));
                ODRIVE_SERIAL.println("w axis1.controller.input_vel " + String(left_vel, 2));
            }
            break;

        case STATE_TORQUE_ESPNOW:
            {
                float throttle_input = espnowData.steering;
                float steering_input = espnowData.throttle;

                AUTO_SERIAL.print("Throttle: " + String(espnowData.throttle, 2) + " ");
                AUTO_SERIAL.println("Steering: " + String(espnowData.steering, 2));
                
                float steering_scale_factor = 1.0 - (STEERING_SENSITIVITY * abs(throttle_input));
                float scaled_steering = steering_input * steering_scale_factor;

                float right_norm = constrain(throttle_input - scaled_steering, -1.0, 1.0);
                float left_norm = constrain(throttle_input + scaled_steering, -1.0, 1.0);

                float curved_right = copysignf(powf(fabsf(right_norm), THROTTLE_EXPO), right_norm);
                float curved_left = copysignf(powf(fabsf(left_norm), THROTTLE_EXPO), left_norm);

                float desired_right_torque = curved_right * MAX_TORQUE;
                float desired_left_torque = curved_left * MAX_TORQUE;

                float right_delta = desired_right_torque - g_last_sent_right_torque;
                float left_delta = desired_left_torque - g_last_sent_left_torque;

                // Ramp only if ABS torque is increasing
                float right_change;
                if (fabsf(desired_right_torque) > fabsf(g_last_sent_right_torque)) {
                    // Torque magnitude is increasing → apply ramp limit
                    right_change = constrain(right_delta, -MAX_TORQUE_CHANGE_PER_CYCLE, MAX_TORQUE_CHANGE_PER_CYCLE);
                } else {
                    // Torque magnitude is decreasing or same → allow instant change
                    right_change = right_delta;
                }

                float left_change;
                if (fabsf(desired_left_torque) > fabsf(g_last_sent_left_torque)) {
                    left_change = constrain(left_delta, -MAX_TORQUE_CHANGE_PER_CYCLE, MAX_TORQUE_CHANGE_PER_CYCLE);
                } else {
                    left_change = left_delta;
                }

                float final_right_torque = g_last_sent_right_torque + right_change;
                float final_left_torque = g_last_sent_left_torque + left_change;

                g_last_sent_right_torque = final_right_torque;
                g_last_sent_left_torque = final_left_torque;


                // float final_right_torque = g_last_sent_right_torque + right_torque_change;
                // float final_left_torque = g_last_sent_left_torque + left_torque_change;

                ODRIVE_SERIAL.print("c 0 " + String(final_right_torque, 2) + "\n");
                ODRIVE_SERIAL.print("c 1 " + String(final_left_torque, 2) + "\n");
                
                // AUTO_SERIAL.print("c 0 " + String(final_right_torque, 2) + " ");
                // AUTO_SERIAL.println("c 1 " + String(final_left_torque, 2) + "\n");

                g_last_sent_right_torque = final_right_torque;
                g_last_sent_left_torque = final_left_torque;
            }
            break;
    }
}
