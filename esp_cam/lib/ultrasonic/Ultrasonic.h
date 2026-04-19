#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

// ESP32-CAM AI-Thinker — using unused SD card pins
#define TRIG_PIN 13
#define ECHO_PIN 14

// Initializes the HC-SR04 ultrasonic sensor pins
void Ultrasonic_Init();

// Takes 6 readings, averages them, returns distance in cm.
// Returns -1.0 if all readings timed out (no object in range).
float Ultrasonic_Read();

#endif
