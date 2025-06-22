/*
 * ESP32 PWM to ODrive Serial (ASCII) Converter with WebSerial
 * * Author: Gemini
 * * Date: June 22, 2025
 * * Version: 2.1 (With WebSerial)
 * *
 * * Description:
 * This code reads two standard PWM signals from an RC receiver and converts them
 * into serial ASCII commands to control the velocity of two axes on an ODrive.
 * It also hosts a WebSerial server over Wi-Fi for wireless debugging.
 * *
 * * How it works:
 * 1.  PWM Input: Uses pin change interrupts to measure incoming RC pulse widths.
 * 2.  Sanity Check & Failsafe: Rejects out-of-range PWM signals and handles
 * signal loss to prevent motor spikes or runaways.
 * 3.  Velocity Mapping: In the main loop, the pulse width (1000-2000µs) is
 * mapped to a velocity range (e.g., -10 to +10 turns/sec).
 * 4.  ODrive Serial Output: Uses a hardware serial port (Serial2) to send
 * ODrive ASCII protocol commands (e.g., "v 0 15.5\n").
 * 5.  WebSerial Monitor: Connects to Wi-Fi and starts a web server. You can
 * connect to it from a browser to see the same debug output that is sent
 * to the USB serial port, allowing for wireless monitoring.
 *
 * Wiring:
 * - RC Receiver:
 * - Connect your first PWM signal source to GPIO 13.
 * - Connect your second PWM signal source to GPIO 12.
 * - Ensure a common ground (GND) between the ESP32 and the receiver.
 * - ODrive:
 * - Connect ESP32 GPIO 17 (TX2) to the ODrive's RX pin.
 * - Connect ESP32 GPIO 16 (RX2) to the ODrive's TX pin.
 * - Ensure a common ground (GND) between the ESP32 and the ODrive.
 * - USB (for Debugging/Flashing):
 * - Connect the ESP32 to your computer via USB.
 */

// Include necessary libraries
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

//================================================================================
// Configuration
//================================================================================

// ★★★ NEW: WebSerial & Wi-Fi Config ★★★
const char* ssid = "YOUR_WIFI_SSID";     // <-- Enter your Wi-Fi network name here
const char* password = "YOUR_WIFI_PASSWORD"; // <-- Enter your Wi-Fi password here
AsyncWebServer server(80); // Create AsyncWebServer object on port 80

// --- PWM Input Pin Definitions ---
const int PWM_INPUT_1 = 13; // For Axis 0
const int PWM_INPUT_2 = 12; // For Axis 1

// --- ODrive Serial Port Configuration ---
#define ODRIVE_SERIAL Serial2
const int ODRIVE_RX_PIN = 16;
const int ODRIVE_TX_PIN = 17;
const long ODRIVE_BAUD_RATE = 115200;

// --- Control & Timing Configuration ---
const unsigned int DEFAULT_PULSE_US = 1500;
const unsigned int MIN_PULSE_US = 1000;
const unsigned int MAX_PULSE_US = 2000;
const int PULSE_DEADZONE_US = 25;

const float MAX_VELOCITY = 10.0;

const unsigned long COMMAND_INTERVAL_MS = 20;
const unsigned long FAILSAFE_TIMEOUT_US = 100000;

//================================================================================
// Global Variables
//================================================================================

volatile unsigned long g_pulse1_start_time = 0;
volatile unsigned long g_pulse2_start_time = 0;
volatile unsigned int g_pulse1_width_us = DEFAULT_PULSE_US;
volatile unsigned int g_pulse2_width_us = DEFAULT_PULSE_US;
volatile unsigned long g_last_pulse1_update_time = 0;
volatile unsigned long g_last_pulse2_update_time = 0;

unsigned long g_last_command_time = 0;

//================================================================================
// Interrupt Service Routines (ISRs) for PWM Input
//================================================================================

void IRAM_ATTR isr_pwm1() {
  if (digitalRead(PWM_INPUT_1) == HIGH) {
    g_pulse1_start_time = micros();
  } else {
    g_pulse1_width_us = micros() - g_pulse1_start_time;
    g_last_pulse1_update_time = micros();
  }
}

void IRAM_ATTR isr_pwm2() {
  if (digitalRead(PWM_INPUT_2) == HIGH) {
    g_pulse2_start_time = micros();
  } else {
    g_pulse2_width_us = micros() - g_pulse2_start_time;
    g_last_pulse2_update_time = micros();
  }
}

//================================================================================
// Helper Functions
//================================================================================

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ★★★ NEW: Callback function for handling messages from the WebSerial client ★★★
void onWebSerialEvent(uint8_t *data, size_t len) {
  WebSerial.println("Received...");
  String msg = "";
  for (size_t i = 0; i < len; i++) {
    msg += (char)data[i];
  }
  WebSerial.print("Message from web: ");
  WebSerial.println(msg);
  Serial.print("Message from web: "); // Also print to USB for good measure
  Serial.println(msg);
}


//================================================================================
// Main Program: setup() and loop()
//================================================================================

void setup() {
  // Start serial for debugging
  Serial.begin(115200);
  Serial.println("Starting PWM to ODrive Serial Converter...");

  // Start serial for ODrive communication
  ODRIVE_SERIAL.begin(ODRIVE_BAUD_RATE, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);
  Serial.println("ODrive serial port initialized.");

  // ★★★ NEW: Wi-Fi and WebSerial Setup ★★★
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Attach the WebSerial interface to the web server
  WebSerial.begin(&server);
  // Set up a callback for when messages are received
  WebSerial.onMessage(onWebSerialEvent);
  // Start the server
  server.begin();
  WebSerial.println("WebSerial is ready! Connect to this IP and go to /webserial");
  // ★★★ End of WebSerial Setup ★★★

  // Configure PWM input pins
  pinMode(PWM_INPUT_1, INPUT_PULLUP);
  pinMode(PWM_INPUT_2, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_1), isr_pwm1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_2), isr_pwm2, CHANGE);
  
  Serial.println("Ready for RC input.");
}

void loop() {
  unsigned long current_millis = millis();

  // Only send commands at the specified interval
  if (current_millis - g_last_command_time >= COMMAND_INTERVAL_MS) {
    g_last_command_time = current_millis;

    // --- Get latest pulse width data safely ---
    unsigned long current_micros = micros();
    noInterrupts();
    unsigned int pulse1 = g_pulse1_width_us;
    unsigned int pulse2 = g_pulse2_width_us;
    unsigned long last_pulse1_time = g_last_pulse1_update_time;
    unsigned long last_pulse2_time = g_last_pulse2_update_time;
    interrupts();

    // --- Apply Failsafe ---
    if (current_micros - last_pulse1_time > FAILSAFE_TIMEOUT_US) {
      pulse1 = DEFAULT_PULSE_US;
    }
    if (current_micros - last_pulse2_time > FAILSAFE_TIMEOUT_US) {
      pulse2 = DEFAULT_PULSE_US;
    }

    // --- Sanity Check ---
    if (pulse1 < MIN_PULSE_US || pulse1 > MAX_PULSE_US) {
      pulse1 = DEFAULT_PULSE_US;
    }
    if (pulse2 < MIN_PULSE_US || pulse2 > MAX_PULSE_US) {
      pulse2 = DEFAULT_PULSE_US;
    }

    // --- Apply Deadzone ---
    if (abs((int)pulse1 - (int)DEFAULT_PULSE_US) <= PULSE_DEADZONE_US) {
      pulse1 = DEFAULT_PULSE_US;
    }
    if (abs((int)pulse2 - (int)DEFAULT_PULSE_US) <= PULSE_DEADZONE_US) {
      pulse2 = DEFAULT_PULSE_US;
    }

    // --- Map Pulse Width to Velocity ---
    float velocity1 = map_float(pulse1, MIN_PULSE_US, MAX_PULSE_US, -MAX_VELOCITY, MAX_VELOCITY);
    float velocity2 = map_float(pulse2, MIN_PULSE_US, MAX_PULSE_US, -MAX_VELOCITY, MAX_VELOCITY);

    // --- Final Safety Clamp ---
    velocity1 = constrain(velocity1, -MAX_VELOCITY, MAX_VELOCITY);
    velocity2 = constrain(velocity2, -MAX_VELOCITY, MAX_VELOCITY);
    
    // --- Format and Send Serial Commands to ODrive ---
    String command1 = "v 0 " + String(velocity1, 2) + "\n";
    String command2 = "v 1 " + String(velocity2, 2) + "\n";

    ODRIVE_SERIAL.print(command1);
    ODRIVE_SERIAL.print(command2);
    
    // --- Send debug info to USB and WebSerial ---
    String debug_msg = "Axis0: " + String(velocity1, 2) + " | Axis1: " + String(velocity2, 2);
    Serial.println(debug_msg);
    WebSerial.println(debug_msg); // ★★★ NEW: Mirror output to WebSerial ★★★
  }
}