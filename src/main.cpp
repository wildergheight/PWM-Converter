/*
 * ESP32 Dual PWM to Time-Spaced PWM Converter for ODrive
 * * Author: Gemini
 * Date: June 13, 2025
 * * Description:
 * This code reads two standard, potentially overlapping PWM signals (like those from an
 * RC receiver) and converts them into two non-overlapping, time-spaced PWM signals.
 * This is useful for motor controllers like the ODrive, which require a gap between
 * the falling edge of one pulse and the rising edge of the next.
 * * How it works:
 * 1. Interrupts: It uses pin change interrupts (`CHANGE`) on two input pins to 
 * asynchronously measure the pulse width of the incoming signals. This is non-blocking
 * and accurate. The results are stored in volatile variables.
 * 2. Failsafe: If an input signal is lost for more than a defined timeout 
 * (FAILSAFE_TIMEOUT_US), the code reverts that channel's output to a neutral/default
 * pulse width (DEFAULT_PULSE_US).
 * 3. State Machine: The main loop runs a simple, non-blocking state machine that
 * generates the new pulses manually. It fires the first pulse, waits for it to finish,
 * fires the second pulse, and then waits for the next cycle. This guarantees the
 * pulses are sequential and never overlap.
 * 4. IRAM_ATTR: The Interrupt Service Routines (ISRs) are placed in IRAM (Instruction RAM)
 * for faster execution, which is a best practice on the ESP32.
 *
 * Wiring:
 * - Connect your first PWM signal source to GPIO 16 (PWM_INPUT_1).
 * - Connect your second PWM signal source to GPIO 17 (PWM_INPUT_2).
 * - Connect ODrive's first PWM input to GPIO 18 (PWM_OUTPUT_1).
 * - Connect ODrive's second PWM input to GPIO 19 (PWM_OUTPUT_2).
 * - Ensure a common ground (GND) between the ESP32 and your signal sources.
 * * You can easily change these pins in the "Pin Definitions" section below.
 */

// Include the main Arduino library header. This is essential for .cpp files.
#include <Arduino.h>

//================================================================================
// Configuration
//================================================================================

// --- Pin Definitions ---
const int PWM_INPUT_1 = 16;  // RX2 on many ESP32 boards
const int PWM_INPUT_2 = 17;  // TX2 on many ESP32 boards

const int PWM_OUTPUT_1 = 18; // Any free GPIO
const int PWM_OUTPUT_2 = 19; // Any free GPIO

// --- Timing and Failsafe Configuration ---
// The period for the entire output cycle (Pulse1 + Pulse2 + Idle time) in microseconds.
// 20000 us = 20 ms = 50 Hz, a standard servo frequency.
const unsigned long CYCLE_PERIOD_US = 20000; 

// Default pulse width in microseconds if the signal is lost (failsafe).
// 1500 us is the standard neutral position for RC signals.
const unsigned int DEFAULT_PULSE_US = 1500;

// Maximum time in microseconds without a new pulse before failsafe is triggered.
// 100,000 us = 100 ms.
const unsigned long FAILSAFE_TIMEOUT_US = 100000;


//================================================================================
// Global Variables
//================================================================================

// --- Volatile variables for Interrupt-Main Loop Communication ---
// These are modified by ISRs, so they must be declared 'volatile'.
volatile unsigned long g_pulse1_start_time = 0;
volatile unsigned long g_pulse2_start_time = 0;

volatile unsigned int g_pulse1_width_us = DEFAULT_PULSE_US;
volatile unsigned int g_pulse2_width_us = DEFAULT_PULSE_US;

volatile unsigned long g_last_pulse1_update_time = 0;
volatile unsigned long g_last_pulse2_update_time = 0;


// --- State machine variables ---
enum State {
  STATE_IDLE,
  STATE_PULSE_1_ON,
  STATE_PULSE_2_ON
};

State g_current_state = STATE_IDLE;
unsigned long g_cycle_start_time = 0;
unsigned long g_pulse_start_time = 0;


//================================================================================
// Interrupt Service Routines (ISRs)
//================================================================================

// ISR for the first PWM input. `IRAM_ATTR` ensures it runs from fast RAM.
void IRAM_ATTR isr_pwm1() {
  if (digitalRead(PWM_INPUT_1) == HIGH) {
    // Rising edge: record the start time.
    g_pulse1_start_time = micros();
  } else {
    // Falling edge: calculate pulse width and record the update time.
    g_pulse1_width_us = micros() - g_pulse1_start_time;
    g_last_pulse1_update_time = micros();
  }
}

// ISR for the second PWM input.
void IRAM_ATTR isr_pwm2() {
  if (digitalRead(PWM_INPUT_2) == HIGH) {
    // Rising edge: record the start time.
    g_pulse2_start_time = micros();
  } else {
    // Falling edge: calculate pulse width and record the update time.
    g_pulse2_width_us = micros() - g_pulse2_start_time;
    g_last_pulse2_update_time = micros();
  }
}


//================================================================================
// Main Program: setup() and loop()
//================================================================================

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  Serial.println("Starting PWM Time-Spacing Converter...");

  // Configure pin modes
  pinMode(PWM_INPUT_1, INPUT_PULLUP);
  pinMode(PWM_INPUT_2, INPUT_PULLUP);
  pinMode(PWM_OUTPUT_1, OUTPUT);
  pinMode(PWM_OUTPUT_2, OUTPUT);

  // Ensure outputs start in a LOW state
  digitalWrite(PWM_OUTPUT_1, LOW);
  digitalWrite(PWM_OUTPUT_2, LOW);

  // Attach interrupts to the input pins. They will trigger on any voltage change (RISING or FALLING).
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_1), isr_pwm1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_2), isr_pwm2, CHANGE);
  
  // Initialize cycle start time
  g_cycle_start_time = micros();
}

void loop() {
  // Atomically copy volatile variables to local scope to prevent race conditions.
  unsigned long current_time = micros();
  noInterrupts(); // Disable interrupts briefly to safely copy multi-byte variables
  unsigned int pulse1_width = g_pulse1_width_us;
  unsigned int pulse2_width = g_pulse2_width_us;
  unsigned long last_pulse1_time = g_last_pulse1_update_time;
  unsigned long last_pulse2_time = g_last_pulse2_update_time;
  interrupts(); // Re-enable interrupts immediately

  // --- Failsafe Check ---
  // If we haven't received a pulse in a while, revert to the default neutral value.
  if (current_time - last_pulse1_time > FAILSAFE_TIMEOUT_US) {
    pulse1_width = DEFAULT_PULSE_US;
  }
  if (current_time - last_pulse2_time > FAILSAFE_TIMEOUT_US) {
    pulse2_width = DEFAULT_PULSE_US;
  }
  
  // --- Non-Blocking State Machine for Pulse Generation ---
  switch (g_current_state) {
    
    case STATE_IDLE:
      // We are waiting for the next cycle to begin.
      if (current_time - g_cycle_start_time >= CYCLE_PERIOD_US) {
        g_cycle_start_time = current_time; // Start a new cycle
        g_pulse_start_time = current_time; // Record when the pulse itself starts
        
        // Begin the first pulse
        digitalWrite(PWM_OUTPUT_1, HIGH);
        g_current_state = STATE_PULSE_1_ON;
      }
      break;

    case STATE_PULSE_1_ON:
      // The first pulse is currently HIGH. Check if it's time to turn it off.
      if (current_time - g_pulse_start_time >= pulse1_width) {
        digitalWrite(PWM_OUTPUT_1, LOW); // End the first pulse
        g_pulse_start_time = current_time; // Record start time for the second pulse
        
        // Immediately begin the second pulse
        digitalWrite(PWM_OUTPUT_2, HIGH);
        g_current_state = STATE_PULSE_2_ON;
      }
      break;

    case STATE_PULSE_2_ON:
      // The second pulse is currently HIGH. Check if it's time to turn it off.
      if (current_time - g_pulse_start_time >= pulse2_width) {
        digitalWrite(PWM_OUTPUT_2, LOW); // End the second pulse
        
        // Return to idle state to wait for the next cycle period.
        g_current_state = STATE_IDLE;
      }
      break;
  }

  // Optional: Print measured pulse widths to Serial Monitor for debugging.
  // Uncomment the following lines to enable. This may affect timing slightly.
  /*
  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 100) {
    Serial.printf("Input 1: %u us, Input 2: %u us\n", pulse1_width, pulse2_width);
    last_print_time = millis();
  }
  */
}
