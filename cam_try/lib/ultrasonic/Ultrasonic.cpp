#include "Ultrasonic.h"

// Maximum distance we care about (400cm = 4m, HC-SR04 max range)
#define MAX_DISTANCE_CM 400
// Timeout in microseconds for pulseIn (400cm × 58 µs/cm = 23200 µs, add margin)
#define PULSE_TIMEOUT_US 24000

// Number of samples for averaging
#define NUM_SAMPLES 2

// Delay between consecutive readings (ms)
#define SAMPLE_DELAY_MS 5

// ========================
// INIT
// ========================
void Ultrasonic_Init() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
    Serial.println("Ultrasonic sensor initialized (TRIG=GPIO13, ECHO=GPIO14).");
}

// ========================
// SINGLE READING
// ========================
static float readSingle() {
    // Send 10µs trigger pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Measure echo pulse duration
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT_US);

    if (duration == 0) {
        return -1.0f; // Timeout — no echo received
    }

    // Convert to cm: speed of sound ~343 m/s → 0.0343 cm/µs
    // Distance = (duration × 0.0343) / 2
    float distance = (duration * 0.0343f) / 2.0f;

    if (distance > MAX_DISTANCE_CM) {
        return -1.0f; // Out of range
    }

    return distance;
}

// ========================
// AVERAGED READING (6 samples)
// ========================
float Ultrasonic_Read() {
    float sum = 0.0f;
    int validCount = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        float d = readSingle();
        if (d > 0) {
            sum += d;
            validCount++;
        }
        delay(SAMPLE_DELAY_MS);
    }

    if (validCount == 0) {
        Serial.println("Ultrasonic: All 6 readings timed out!");
        return -1.0f;
    }

    float avg = sum / validCount;
    Serial.printf("Ultrasonic: %.2f cm (avg of %d/%d valid readings)\n", avg, validCount, NUM_SAMPLES);
    return avg;
}
