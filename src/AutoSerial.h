#pragma once

/*
 * AutoSerial.h
 * Parses USB ("automation") serial commands of the form "R 0.5,L -0.3"
 * used to override ESP-NOW control with velocity commands from a host PC.
 */

#include <Arduino.h>

// Reads any available bytes on AUTO_SERIAL, buffers until newline, then parses.
void checkAutoSerial();
