#pragma once

/*
 * EspNowHandler.h
 * ESP-NOW init, receive callback, and channel-hunting logic used to
 * re-acquire the remote link if packets stop arriving.
 */

#include <Arduino.h>

// Initializes WiFi station mode + ESP-NOW and registers the receive callback.
// Call once from setup(). Prints status/MAC info to AUTO_SERIAL.
void initEspNow();

// If we haven't heard from the remote in SCAN_START_DELAY_MS, hop to the next
// WiFi channel every SCAN_INTERVAL_MS to hunt for the remote's channel.
// void handleChannelScanning();

// Broadcasts current cart status (control state + faults + voltage) to the
// tag so the golfer can feel it via vibration motor without turning around.
void sendStatusAlert();

// Prints one UWBLOG CSV line to AUTO_SERIAL if a new UWB packet arrived
// since the last call. Call every loop() iteration. No-op if UWB_EVAL_LOG
// is false or no new packet is pending. Keeps Serial I/O out of the
// ESP-NOW recv callback itself.
void checkUwbEvalLog();