#ifndef QR_READER_H
#define QR_READER_H

#include <Arduino.h>

// Initializes the ESP32-CAM and the QR Code Reader
void QR_Reader_Init();

// Scans for a QR code and returns the text if found, or an empty string if nothing was found this frame
String QR_Reader_Scan();

#endif
