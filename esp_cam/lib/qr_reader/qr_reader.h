#ifndef QR_READER_H
#define QR_READER_H

#include <Arduino.h>
#include "freertos/semphr.h"

// Initializes the camera hardware and quirc decoder.
// Must be called once in setup().
void QR_Reader_Init();

// Non-blocking: sets the scan_requested flag.
// Call from loop() — do NOT block loop() waiting for the result.
void QR_Reader_RequestScan();

// Call this every loop() iteration.
// Returns true (once) when a scan result is ready.
// Retrieve result with QR_Reader_GetResult().
bool QR_Reader_ResultReady();

// Returns the last decoded QR payload string ("red", "green", "blue", or "").
String QR_Reader_GetResult();

// Returns true while a scan task is running (camera is in grayscale mode).
bool QR_Reader_IsScanning();

// Exposes the camera mutex so the HTTP capture handler can block during scans.
SemaphoreHandle_t QR_Reader_GetMutex();

#endif // QR_READER_H
