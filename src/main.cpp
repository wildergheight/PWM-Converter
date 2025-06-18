/*
 * ESP32 PWM to ODrive Serial (ASCII) Converter
 * * Author: Gemini
 * Date: June 17, 2025
 * * Description:
 * This code reads two standard PWM signals from an RC receiver and converts them
 * into serial ASCII commands to control the velocity of two axes on an ODrive.
 * This provides more precise and robust control than PWM output and is ideal
 * for future automation.
 * * How it works:
 * 1.  PWM Input: Uses pin change interrupts to measure the incoming RC pulse widths
 * asynchronously, just like the previous version.
 * 2.  Failsafe & Deadzone: Includes failsafe to handle signal loss and a deadzone
 * around the neutral signal to prevent motor jitter.
 * 3.  Velocity Mapping: In the main loop, the measured pulse width (1000-2000µs)
 * is mapped to a velocity range (e.g., -20 to +20 turns/sec).
 * 4.  Serial Command Output: The ESP32 uses a second hardware serial port to send
 * ODrive ASCII protocol commands (e.g., "v 0 15.5\n") at a fixed rate. This
 * avoids flooding the ODrive with commands while maintaining responsive control.
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
 * - USB (for Debugging):
 * - Connect the ESP32 to your computer via USB to view debug messages.
 *
 * ** IMPORTANT ODRIVE CONFIGURATION **
 * You must configure the ODrive's GPIO pins for UART communication. For example:
 * odrv0.config.enable_uart_a = True
 * odrv0.config.gpio(1)_mode = GpioMode.UART_A // Use your ODrive's correct TX pin
 * odrv0.config.gpio(2)_mode = GpioMode.UART_A // Use your ODrive's correct RX pin
 * odrv0.save_configuration()
 * And ensure the baud rate matches `ODRIVE_BAUD_RATE` below.
 */

// Include the main Arduino library header.
#include <Arduino.h>

//================================================================================
// Configuration
//================================================================================

// --- PWM Input Pin Definitions ---
const int PWM_INPUT_1 = 13; // For Axis 0
const int PWM_INPUT_2 = 12; // For Axis 1

// --- ODrive Serial Port Configuration ---
// Using Serial2 for ODrive communication. Serial (UART0) is for USB debugging.
#define ODRIVE_SERIAL Serial2
const int ODRIVE_RX_PIN = 16; // GPIO 16 (ESP32 RX2) -> ODrive TX
const int ODRIVE_TX_PIN = 17; // GPIO 17 (ESP32 TX2) -> ODrive RX
const long ODRIVE_BAUD_RATE = 115200;

// --- Control & Timing Configuration ---
const unsigned int DEFAULT_PULSE_US = 1500; // Neutral RC signal
const unsigned int MIN_PULSE_US = 1000;     // Full reverse RC signal
const unsigned int MAX_PULSE_US = 2000;     // Full forward RC signal
const int PULSE_DEADZONE_US = 25;           // Jitter prevention range (+/-)

const float MAX_VELOCITY = 20.0; // Max velocity in [turns/sec] at full stick

// How often to send commands to ODrive (in milliseconds). 20ms = 50 Hz.
const unsigned long COMMAND_INTERVAL_MS = 20;
const unsigned long FAILSAFE_TIMEOUT_US = 100000; // 100 ms

//================================================================================
// Global Variables
//================================================================================

// For PWM input interrupts
volatile unsigned long g_pulse1_start_time = 0;
volatile unsigned long g_pulse2_start_time = 0;
volatile unsigned int g_pulse1_width_us = DEFAULT_PULSE_US;
volatile unsigned int g_pulse2_width_us = DEFAULT_PULSE_US;
volatile unsigned long g_last_pulse1_update_time = 0;
volatile unsigned long g_last_pulse2_update_time = 0;

// For timing the serial commands
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
// Helper Function
//================================================================================

// Maps an input value from a source range to a target range (float version)
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

    // Clamp values to the max velocity in case of over/under-shoot from RC receiver
    velocity1 = constrain(velocity1, -MAX_VELOCITY, MAX_VELOCITY);
    velocity2 = constrain(velocity2, -MAX_VELOCITY, MAX_VELOCITY);
    
    // --- Format and Send Serial Commands to ODrive ---
    String command1 = "v 0 " + String(velocity1, 2) + "\n"; // "v 0 -15.75\n"
    String command2 = "v 1 " + String(velocity2, 2) + "\n"; // "v 1 20.00\n"

    ODRIVE_SERIAL.print(command1);
    ODRIVE_SERIAL.print(command2);
    
    // --- Optional: Print debug info to USB serial ---
    // Uncomment the line below for debugging.
    Serial.print("Axis0: " + String(velocity1, 2) + " | Axis1: " + String(velocity2, 2));
  }
}
