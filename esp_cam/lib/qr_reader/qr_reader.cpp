// ============================================================
// qr_reader.cpp  —  ESP32-CAM QR decoder using quirc directly
//
// Architecture matches cam_try exactly:
//   - QR_Reader_RequestScan() sets a flag and returns immediately.
//   - loop() calls this + QR_Reader_ResultReady() each iteration.
//   - The FreeRTOS scan task runs on Core 0.
//   - The camera mutex BLOCKS the HTTP capture handler during scanning,
//     just like cam_try's stream_handler checks is_scanning.
// ============================================================

#include "qr_reader.h"
#include "esp_camera.h"

extern "C" {
    #include "quirc/quirc.h"
}

// ── Pin map (AI-Thinker ESP32-CAM) ───────────────────────────────────────────
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// ── Module-level state ────────────────────────────────────────────────────────
static struct quirc      *s_quirc        = NULL;
static camera_config_t    s_jpeg_cfg;            // saved for JPEG reinit
static SemaphoreHandle_t  s_cam_mutex    = NULL;

// ── Scan state flags ──────────────────────────────────────────────────────────
// s_scan_running  : true from the moment scan is requested until task finishes
//                   (used to block capture_handler — set EARLY in RequestScan)
// s_spawn_pending : true only until the FreeRTOS task has been created
//                   (separate from s_scan_running to avoid the spawn-never-
//                    happens bug where !s_scan_running was always false)
static volatile bool s_spawn_pending  = false;
static volatile bool s_scan_completed = false;
static volatile bool s_scan_running   = false;
static String        s_scan_result    = "";

// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────────────────────
static void fill_jpeg_config(camera_config_t &cfg) {
    cfg.ledc_channel  = LEDC_CHANNEL_0;
    cfg.ledc_timer    = LEDC_TIMER_0;
    cfg.pin_d0        = CAM_PIN_D0;
    cfg.pin_d1        = CAM_PIN_D1;
    cfg.pin_d2        = CAM_PIN_D2;
    cfg.pin_d3        = CAM_PIN_D3;
    cfg.pin_d4        = CAM_PIN_D4;
    cfg.pin_d5        = CAM_PIN_D5;
    cfg.pin_d6        = CAM_PIN_D6;
    cfg.pin_d7        = CAM_PIN_D7;
    cfg.pin_xclk      = CAM_PIN_XCLK;
    cfg.pin_pclk      = CAM_PIN_PCLK;
    cfg.pin_vsync     = CAM_PIN_VSYNC;
    cfg.pin_href      = CAM_PIN_HREF;
    cfg.pin_sccb_sda  = CAM_PIN_SIOD;
    cfg.pin_sccb_scl  = CAM_PIN_SIOC;
    cfg.pin_pwdn      = CAM_PIN_PWDN;
    cfg.pin_reset     = CAM_PIN_RESET;
    cfg.xclk_freq_hz  = 20000000;
    cfg.pixel_format  = PIXFORMAT_JPEG;
    cfg.frame_size    = FRAMESIZE_VGA;
    cfg.jpeg_quality  = 10;
    cfg.fb_count      = 1;
}

// ─────────────────────────────────────────────────────────────────────────────
// restore_jpeg() — bring the camera back to JPEG VGA after a scan.
// Retries up to 3 times with increasing delays + PWDN power-cycle so the
// camera always comes back even after a failed or aborted grayscale scan.
// ─────────────────────────────────────────────────────────────────────────────
static bool restore_jpeg() {
    for (int attempt = 0; attempt < 3; attempt++) {
        esp_camera_deinit();

        if (attempt > 0) {
            // Power-cycle the sensor between retries
            digitalWrite(CAM_PIN_PWDN, HIGH);
            delay(200);
            digitalWrite(CAM_PIN_PWDN, LOW);
            delay(200 + attempt * 100);
        } else {
            delay(100);
        }

        if (esp_camera_init(&s_jpeg_cfg) == ESP_OK) {
            sensor_t *s = esp_camera_sensor_get();
            if (s) { s->set_vflip(s, 1); s->set_hmirror(s, 1); }
            Serial.printf(">>> JPEG restored (attempt %d).\n", attempt + 1);
            return true;
        }
        Serial.printf("!!! JPEG reinit attempt %d failed.\n", attempt + 1);
    }
    Serial.println("!!! Camera could not be restored after 3 attempts.");
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Scan FreeRTOS task — identical to cam_try's qr_scan_task
// Runs on Core 0. loop() runs on Core 1. No contention.
// ─────────────────────────────────────────────────────────────────────────────
static void qr_scan_task(void *pvParameters) {
    Serial.println(">>> QR scan task started (grayscale mode).");
    // s_scan_running was already set true in QR_Reader_RequestScan() before
    // the task was created, so capture_handler blocks immediately.
    // Clear spawn_pending now that we are actually executing.
    s_spawn_pending = false;

    const int W = 320, H = 240;

    // ── Step 1: grab mutex and tear down JPEG camera ──────────────────────────
    xSemaphoreTake(s_cam_mutex, portMAX_DELAY);
    esp_camera_deinit();
    delay(100);

    // ── Step 2: reinit as raw GRAYSCALE QVGA ─────────────────────────────────
    camera_config_t gs = s_jpeg_cfg;   // copy full pin wiring from saved config
    gs.pixel_format = PIXFORMAT_GRAYSCALE;
    gs.frame_size   = FRAMESIZE_QVGA;  // 320×240
    gs.jpeg_quality = 10;
    gs.fb_count     = 1;

    if (esp_camera_init(&gs) != ESP_OK) {
        Serial.println("QR: grayscale init failed");
        s_scan_result = "";
        restore_jpeg();              // retry until camera is back
        xSemaphoreGive(s_cam_mutex);
        s_scan_running   = false;
        s_scan_completed = true;
        vTaskDelete(NULL);
        return;
    }

    // Apply high-contrast sensor settings — critical for quirc (from cam_try)
    sensor_t *sens = esp_camera_sensor_get();
    if (sens) {
        sens->set_contrast(sens, 2);    // maximum contrast
        sens->set_brightness(sens, 1);  // slightly brighter
        sens->set_saturation(sens, 0);  // irrelevant for grayscale
        sens->set_vflip(sens, 1);
        sens->set_hmirror(sens, 1);
    }

    // Flush a few frames so AEC/AGC settle (from cam_try)
    for (int i = 0; i < 4; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        delay(60);
    }

    // ── Step 3: resize quirc to match frame ───────────────────────────────────
    if (quirc_resize(s_quirc, W, H) != 0) {
        Serial.println("QR: quirc_resize failed");
        restore_jpeg();              // retry until camera is back
        xSemaphoreGive(s_cam_mutex);
        s_scan_running   = false;
        s_scan_completed = true;
        vTaskDelete(NULL);
        return;
    }

    // ── Step 4: scan loop (7 seconds) — identical to cam_try ─────────────────
    bool found = false;
    unsigned long t0 = millis();

    while (millis() - t0 < 7000 && !found) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { delay(20); continue; }

        if (fb->width  == (size_t)W &&
            fb->height == (size_t)H &&
            fb->format == PIXFORMAT_GRAYSCALE)
        {
            int qw, qh;
            uint8_t *image = quirc_begin(s_quirc, &qw, &qh);

            if (image && qw == W && qh == H) {
                // Direct copy — no decompression, no colour conversion!
                memcpy(image, fb->buf, (size_t)(W * H));
                quirc_end(s_quirc);

                int num_codes = quirc_count(s_quirc);
                Serial.printf("  frame @%lums: quirc found %d code(s)\n",
                              millis() - t0, num_codes);

                if (num_codes > 0) {
                    struct quirc_code code;
                    struct quirc_data data;
                    quirc_extract(s_quirc, 0, &code);
                    quirc_decode_error_t err = quirc_decode(&code, &data);
                    if (err == QUIRC_SUCCESS) {
                        found = true;
                        s_scan_result = String((char *)data.payload);
                        Serial.println("=> QR Detected: " + s_scan_result);
                    } else {
                        Serial.printf("  decode error: %s\n", quirc_strerror(err));
                    }
                }
            } else {
                quirc_end(s_quirc);  // keep quirc state clean
            }
        }

        esp_camera_fb_return(fb);
        delay(30);
    }

    // ── Step 5: restore JPEG VGA (with retry) ────────────────────────────────
    restore_jpeg();

    xSemaphoreGive(s_cam_mutex);

    if (!found) s_scan_result = "";

    Serial.println(">>> Scan finished. Result: \"" + s_scan_result + "\"");
    s_scan_running   = false;
    s_scan_completed = true;
    vTaskDelete(NULL);
}

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

void QR_Reader_Init() {
    // Flash LED off
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);

    // Power-cycle sensor via PWDN (same as cam_try)
    pinMode(CAM_PIN_PWDN, OUTPUT);
    digitalWrite(CAM_PIN_PWDN, HIGH);
    delay(200);
    digitalWrite(CAM_PIN_PWDN, LOW);
    delay(200);

    s_cam_mutex = xSemaphoreCreateMutex();

    // Allocate quirc handle once at boot
    s_quirc = quirc_new();
    if (!s_quirc) {
        Serial.println("CRITICAL: quirc_new() failed");
        return;
    }

    fill_jpeg_config(s_jpeg_cfg);

    esp_err_t err = esp_camera_init(&s_jpeg_cfg);
    if (err != ESP_OK) {
        Serial.printf("CRITICAL: Camera init failed (0x%x)\n", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_vflip(s, 1);
        s->set_hmirror(s, 1);
    }

    Serial.println("QR_Reader: camera ready (JPEG VGA).");
}

// Non-blocking: flags that a scan is needed.
// Sets s_scan_running=true IMMEDIATELY so capture_handler stops fetching
// frames before the FreeRTOS task even starts.
void QR_Reader_RequestScan() {
    if (!s_scan_running && !s_spawn_pending) {
        s_scan_result    = "";
        s_scan_completed = false;
        s_scan_running   = true;   // block capture_handler right now
        s_spawn_pending  = true;   // signal ResultReady() to spawn the task
    }
}

// Call every loop() iteration.
// Spawns the scan task on first call after RequestScan().
// Returns true (once) when the result is ready.
bool QR_Reader_ResultReady() {
    // Spawn task the first time we see s_spawn_pending
    if (s_spawn_pending) {
        s_spawn_pending = false;   // clear BEFORE create so we don't double-spawn
        xTaskCreatePinnedToCore(
            qr_scan_task,
            "qr_scan",
            20480,          // 20 KB stack — same as cam_try
            NULL,
            1,
            NULL,
            0               // Core 0 — loop() is on Core 1
        );
        return false;
    }

    if (s_scan_completed) {
        s_scan_completed = false;  // consume the flag
        return true;
    }
    return false;
}

String QR_Reader_GetResult() {
    return s_scan_result;
}

bool QR_Reader_IsScanning() {
    return s_scan_running;
}

SemaphoreHandle_t QR_Reader_GetMutex() {
    return s_cam_mutex;
}
