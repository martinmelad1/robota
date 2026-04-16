#include "qr_reader.h"
#include <ESP32QRCodeReader.h>
#include "esp_camera.h"

ESP32QRCodeReader qrReader(CAMERA_MODEL_AI_THINKER, FRAMESIZE_QVGA);

void QR_Reader_Init() {
    // Turn OFF the Flashlight LED to prevent glare on paper
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);

    // Disable verbose debug (set to true if you need to debug again)
    qrReader.setDebug(false);

    // Setup camera hardware
    qrReader.setup();

    // *** CRITICAL FIX #1 ***
    // The library's begin() defaults to Core 0, which shares with UART/WiFi/system tasks.
    // This causes watchdog resets (rst:0x1 POWERON_RESET) and CPU starvation.
    // The library's own official example uses beginOnCore(1).
    qrReader.beginOnCore(1);

    // *** CRITICAL FIX #2 ***
    // Set the sensor orientation ONCE at init, not during scanning.
    // Changing hmirror/vflip while the background task captures frames
    // creates a race condition that corrupts the image.
    sensor_t * s = esp_camera_sensor_get();
    if (s) {
        s->set_contrast(s, 2);     // Max contrast for sharp black/white separation
        s->set_brightness(s, 1);   // Slightly brighter to help in dim rooms
        s->set_hmirror(s, 0);      // No horizontal mirror
        s->set_vflip(s, 0);        // No vertical flip
    }

    Serial.println("QR Reader Initialized (Core 1).");
}

String QR_Reader_Scan() {
    String decodedText = "";

    // Check if the background task found a QR code
    struct QRCodeData qrCodeData;
    if (qrReader.receiveQrCode(&qrCodeData, 500)) {
        if (qrCodeData.valid) {
            decodedText = String((const char *)qrCodeData.payload);
            Serial.println("QR Code Read: " + decodedText);
        } else {
            Serial.print("QR Found but invalid: ");
            Serial.println((const char *)qrCodeData.payload);
        }
    }

    return decodedText;
}
