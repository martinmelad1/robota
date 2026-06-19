#include <Arduino.h>
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_camera.h"
// img_converters.h not needed — grayscale frames go directly to quirc

extern "C" {
    #include "quirc/quirc.h"
}

// ── Server handles ────────────────────────────────────────────────────────────
httpd_handle_t web_httpd    = NULL;
httpd_handle_t stream_httpd = NULL;

// ── Shared state ──────────────────────────────────────────────────────────────
volatile bool scan_requested  = false;
volatile bool scan_completed  = false;
volatile bool is_scanning     = false;
String        last_scan_result = "";
SemaphoreHandle_t cam_mutex;

// ── quirc instance (allocated once at boot) ───────────────────────────────────
struct quirc *q = NULL;

// ─────────────────────────────────────────────────────────────────────────────
// HTML page (stored in flash)
// ─────────────────────────────────────────────────────────────────────────────
static const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>ESP32-CAM QR Scanner</title>
    <style>
        body { font-family: 'Inter', sans-serif; background: #0f172a; color: #f8fafc; margin: 0; padding: 20px; display: flex; flex-direction: column; align-items: center; }
        h2 { margin-bottom: 20px; font-weight: 600; background: linear-gradient(90deg, #38bdf8 0%, #34d399 100%); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }
        .cam-container { position: relative; width: 100%; max-width: 640px; border-radius: 16px; overflow: hidden; box-shadow: 0 10px 30px rgba(0,0,0,0.5); border: 1px solid #334155; background: #000; min-height: 240px; }
        img { width: 100%; height: auto; display: block; }
        .controls { margin-top: 24px; width: 100%; max-width: 640px; display: flex; flex-direction: column; gap: 16px; }
        button { background: linear-gradient(135deg, #6366f1 0%, #a855f7 100%); color: white; border: none; padding: 16px 24px; font-size: 18px; font-weight: 600; border-radius: 12px; cursor: pointer; transition: transform 0.2s, box-shadow 0.2s; display: flex; justify-content: center; align-items: center; }
        button:hover { transform: translateY(-2px); box-shadow: 0 8px 20px rgba(99, 102, 241, 0.4); }
        button:active { transform: translateY(0); }
        .result-card { background: rgba(255,255,255,0.05); backdrop-filter: blur(10px); padding: 20px; border-radius: 12px; border: 1px solid rgba(255,255,255,0.1); display: none; }
        .result-title { font-size: 12px; color: #94a3b8; margin-bottom: 8px; text-transform: uppercase; letter-spacing: 1px; font-weight: bold; }
        .result-data { font-size: 20px; font-family: monospace; word-break: break-all; color: #4ade80; }
        .loader { border: 3px solid rgba(255,255,255,0.1); border-top: 3px solid #fff; border-radius: 50%; width: 20px; height: 20px; animation: spin 1s linear infinite; display: none; }
        @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }
    </style>
</head>
<body>
    <h2>Robota QR Scanner</h2>
    <div class="cam-container">
        <img id="stream" src="" />
    </div>
    <div class="controls">
        <button id="scanBtn" onclick="scanQR()">
            <span id="btnText">Scan QR Code</span>
            <div id="loader" class="loader"></div>
        </button>
        <div id="resultCard" class="result-card">
            <div class="result-title">Scanned Payload</div>
            <div id="resultData" class="result-data"></div>
        </div>
    </div>
    <script>
        document.getElementById('stream').src = 'http://' + window.location.hostname + ':81/stream';

        async function scanQR() {
            const btn      = document.getElementById('scanBtn');
            const btnText  = document.getElementById('btnText');
            const loader   = document.getElementById('loader');
            const resultCard = document.getElementById('resultCard');
            const resultData = document.getElementById('resultData');

            btn.disabled = true;
            btnText.style.display = 'none';
            loader.style.display  = 'block';
            resultCard.style.display = 'none';

            try {
                const response = await fetch('/scan', { signal: AbortSignal.timeout(15000) });
                const text = await response.text();
                resultData.innerText = text;
                resultCard.style.display = 'block';
                resultData.style.color = (text === "No QR code found" || text === "Camera Error")
                    ? "#f87171" : "#4ade80";
            } catch(e) {
                resultData.innerText = "Error communicating with camera.";
                resultData.style.color = "#f87171";
                resultCard.style.display = 'block';
            }

            btn.disabled = false;
            btnText.style.display = 'block';
            loader.style.display  = 'none';
        }
    </script>
</body>
</html>
)rawliteral";

// ─────────────────────────────────────────────────────────────────────────────
// Stream constants
// ─────────────────────────────────────────────────────────────────────────────
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ─────────────────────────────────────────────────────────────────────────────
// Camera init / deinit helpers
// ─────────────────────────────────────────────────────────────────────────────
static camera_config_t cam_config;  // kept globally so we can re-use it

bool init_camera() {
    // Power-cycle the camera sensor via PWDN pin
    pinMode(32, OUTPUT);
    digitalWrite(32, HIGH);
    delay(200);
    digitalWrite(32, LOW);
    delay(200);

    cam_config.ledc_channel  = LEDC_CHANNEL_0;
    cam_config.ledc_timer    = LEDC_TIMER_0;
    cam_config.pin_d0        = 5;
    cam_config.pin_d1        = 18;
    cam_config.pin_d2        = 19;
    cam_config.pin_d3        = 21;
    cam_config.pin_d4        = 36;
    cam_config.pin_d5        = 39;
    cam_config.pin_d6        = 34;
    cam_config.pin_d7        = 35;
    cam_config.pin_xclk      = 0;
    cam_config.pin_pclk      = 22;
    cam_config.pin_vsync     = 25;
    cam_config.pin_href      = 23;
    cam_config.pin_sccb_sda  = 26;
    cam_config.pin_sccb_scl  = 27;
    cam_config.pin_pwdn      = 32;
    cam_config.pin_reset     = -1;
    cam_config.xclk_freq_hz  = 20000000;
    cam_config.pixel_format  = PIXFORMAT_JPEG;
    cam_config.frame_size    = FRAMESIZE_VGA;
    cam_config.jpeg_quality  = 10;
    cam_config.fb_count      = 1;

    esp_err_t err = esp_camera_init(&cam_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_vflip(s, 1);
        s->set_hmirror(s, 1);
    }
    Serial.println("Camera init OK.");
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// HTTP handlers
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html, strlen(index_html));
}

esp_err_t scan_handler(httpd_req_t *req) {
    if (is_scanning) {
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, "Busy", 4);
    }

    scan_requested = true;
    scan_completed = false;

    unsigned long start = millis();
    while (!scan_completed) {
        delay(50);
        if (millis() - start > 12000) break;  // 12 s timeout
    }

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, last_scan_result.c_str(), last_scan_result.length());
}

esp_err_t stream_handler(httpd_req_t *req) {
    esp_err_t res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;

    char part_buf[64];

    while (true) {
        if (is_scanning) {
            delay(100);
            continue;
        }

        xSemaphoreTake(cam_mutex, portMAX_DELAY);
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            xSemaphoreGive(cam_mutex);
            delay(100);
            continue;
        }

        size_t hlen = snprintf(part_buf, 64, _STREAM_PART, fb->len);
        res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));

        esp_camera_fb_return(fb);
        xSemaphoreGive(cam_mutex);

        if (res != ESP_OK) break;
    }
    return res;
}

// ─────────────────────────────────────────────────────────────────────────────
// QR scan task  ← dedicated FreeRTOS task (20 KB stack, core 0)
//
// Strategy: deinit camera → reinit as RAW GRAYSCALE → scan → deinit → reinit
// as JPEG.  Raw grayscale feeds quirc with zero JPEG artefacts, which is the
// main reason quirc fails on compressed frames.
// ─────────────────────────────────────────────────────────────────────────────
void qr_scan_task(void *pvParameters) {
    Serial.println(">>> QR scan task started (grayscale mode).");
    is_scanning = true;

    const int W = 320, H = 240;

    // ── Step 1: grab the mutex and tear down the JPEG camera ─────────────────
    xSemaphoreTake(cam_mutex, portMAX_DELAY);

    esp_camera_deinit();
    delay(100);

    // ── Step 2: reinit in raw GRAYSCALE mode ──────────────────────────────────
    camera_config_t gs_cfg = cam_config;        // copy base pin wiring
    gs_cfg.pixel_format = PIXFORMAT_GRAYSCALE;
    gs_cfg.frame_size   = FRAMESIZE_QVGA;       // 320×240
    gs_cfg.jpeg_quality = 10;                   // ignored for grayscale
    gs_cfg.fb_count     = 1;

    if (esp_camera_init(&gs_cfg) != ESP_OK) {
        Serial.println("Error: grayscale camera init failed.");
        last_scan_result = "Camera Error";
        xSemaphoreGive(cam_mutex);
        // Try to bring JPEG back anyway
        esp_camera_deinit();
        delay(100);
        esp_camera_init(&cam_config);
        is_scanning    = false;
        scan_completed = true;
        scan_requested = false;
        vTaskDelete(NULL);
        return;
    }

    // Apply high-contrast sensor settings — critical for quirc
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_contrast(s, 2);    // maximum contrast
        s->set_brightness(s, 1);  // slightly brighter
        s->set_saturation(s, 0);  // irrelevant for grayscale but keep clean
        s->set_vflip(s, 1);
        s->set_hmirror(s, 1);
    }

    // Flush a few frames so AEC/AGC settle
    for (int i = 0; i < 4; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        delay(60);
    }

    // ── Step 3: resize quirc to match frame exactly ───────────────────────────
    if (quirc_resize(q, W, H) != 0) {
        Serial.println("Error: quirc_resize failed.");
        last_scan_result = "Camera Error";
        esp_camera_deinit();
        delay(100);
        esp_camera_init(&cam_config);
        xSemaphoreGive(cam_mutex);
        is_scanning    = false;
        scan_completed = true;
        scan_requested = false;
        vTaskDelete(NULL);
        return;
    }

    // ── Step 4: scan loop (up to 7 seconds) ───────────────────────────────────
    bool found = false;
    unsigned long t0 = millis();

    while (millis() - t0 < 7000 && !found) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { delay(20); continue; }

        if (fb->width == (size_t)W && fb->height == (size_t)H &&
            fb->format == PIXFORMAT_GRAYSCALE)
        {
            int qw, qh;
            uint8_t *image = quirc_begin(q, &qw, &qh);

            if (image && qw == W && qh == H) {
                // Direct copy — no decompression, no colour conversion!
                memcpy(image, fb->buf, (size_t)(W * H));
                quirc_end(q);

                int num_codes = quirc_count(q);
                Serial.printf("   frame @ %lums: quirc found %d code(s)\n",
                              millis() - t0, num_codes);

                if (num_codes > 0) {
                    struct quirc_code code;
                    struct quirc_data data;
                    quirc_extract(q, 0, &code);
                    quirc_decode_error_t err = quirc_decode(&code, &data);
                    if (err == QUIRC_SUCCESS) {
                        found = true;
                        last_scan_result = String((char *)data.payload);
                        Serial.println("=> QR Detected: " + last_scan_result);
                    } else {
                        Serial.printf("   decode error: %s\n", quirc_strerror(err));
                    }
                }
            } else {
                // quirc_begin returned NULL or wrong dims — still call end
                quirc_end(q);
            }
        }

        esp_camera_fb_return(fb);
        delay(30);
    }

    // ── Step 5: tear down grayscale, bring back JPEG VGA ─────────────────────
    esp_camera_deinit();
    delay(100);

    if (esp_camera_init(&cam_config) == ESP_OK) {
        sensor_t *s2 = esp_camera_sensor_get();
        if (s2) {
            s2->set_vflip(s2, 1);
            s2->set_hmirror(s2, 1);
        }
        Serial.println(">>> JPEG stream restored.");
    } else {
        Serial.println("!!! Failed to restore JPEG camera.");
    }

    xSemaphoreGive(cam_mutex);

    if (!found) last_scan_result = "No QR code found";

    Serial.println(">>> Scan finished.");
    is_scanning    = false;
    scan_completed = true;
    scan_requested = false;
    vTaskDelete(NULL);
}

// ─────────────────────────────────────────────────────────────────────────────
// HTTP server startup
// ─────────────────────────────────────────────────────────────────────────────
void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.server_port = 80;
    httpd_uri_t index_uri = { .uri = "/",     .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
    httpd_uri_t scan_uri  = { .uri = "/scan", .method = HTTP_GET, .handler = scan_handler,  .user_ctx = NULL };

    if (httpd_start(&web_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(web_httpd, &index_uri);
        httpd_register_uri_handler(web_httpd, &scan_uri);
    }

    config.server_port += 1;
    config.ctrl_port   += 1;
    httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };

    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Arduino entry points
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("\nStarting ESP32-CAM QR Scanner...");

    // Flash LED off
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);

    cam_mutex = xSemaphoreCreateMutex();

    // Allocate quirc once at boot (only the handle; resize happens per-scan)
    q = quirc_new();
    if (!q) {
        Serial.println("CRITICAL: quirc_new() failed.");
        return;
    }

    if (!init_camera()) {
        Serial.println("CRITICAL ERROR: Camera init failed. Unplug USB and reconnect!");
        return;
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP32_CAM_TEST", "12345678");
    delay(100);

    startCameraServer();

    Serial.println("=====================================");
    Serial.println("Connect to WiFi: ESP32_CAM_TEST");
    Serial.println("Password      : 12345678");
    Serial.println("Open browser  : http://192.168.4.1");
    Serial.println("=====================================");
}

void loop() {
    if (scan_requested && !is_scanning) {
        // Spawn scan on a dedicated task with 20 KB stack on core 0
        // loop() runs on core 1 by default, so this avoids any contention
        xTaskCreatePinnedToCore(
            qr_scan_task,   // task function
            "qr_scan",      // name
            20480,          // stack size in bytes  ← THE KEY FIX
            NULL,           // parameter
            1,              // priority
            NULL,           // handle (don't need it)
            0               // core 0
        );
    }
    delay(50);
}