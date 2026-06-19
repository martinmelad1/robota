// ==========================================
// ESP32-CAM: QR + Ultrasonic + UART + WiFi Camera Server
//
// Architecture mirrors cam_try exactly:
//   - MJPEG stream_handler (port 81) pauses itself while a QR scan is running,
//     causing the browser's <img src="/stream"> to freeze on the last frame.
//   - capture_handler (port 80 /capture) returns 503 during scans.
//   - loop() is NEVER blocked — uses QR_Reader_RequestScan() / ResultReady().
// ==========================================
#include <Arduino.h>
#include "qr_reader.h"
#include "UART_slave_cam.h"
#include "Ultrasonic.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_camera.h"

// Join the BASE's WiFi network as a client
const char* base_ssid     = "RobotController";
const char* base_password = "robot1234";

httpd_handle_t cam_httpd    = NULL;
httpd_handle_t stream_httpd = NULL;

// ─────────────────────────────────────────────────────────────────────────────
// MJPEG stream constants  (identical to cam_try)
// ─────────────────────────────────────────────────────────────────────────────
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ─────────────────────────────────────────────────────────────────────────────
// MJPEG stream handler — identical to cam_try's stream_handler.
//
// While a QR scan is running the handler pauses (yields 100 ms).
// This causes the browser's <img src="http://CAM_IP:81/stream"> to
// freeze on the last frame — exactly the desired behaviour.
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t stream_handler(httpd_req_t *req) {
    esp_err_t res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;

    char part_buf[64];

    while (true) {
        // Mirror cam_try: pause stream while scan task owns the camera
        if (QR_Reader_IsScanning()) {
            delay(100);
            continue;
        }

        SemaphoreHandle_t mtx = QR_Reader_GetMutex();
        xSemaphoreTake(mtx, portMAX_DELAY);
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            xSemaphoreGive(mtx);
            delay(100);
            continue;
        }

        size_t hlen = snprintf(part_buf, 64, _STREAM_PART, fb->len);
        res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));

        esp_camera_fb_return(fb);
        xSemaphoreGive(mtx);

        if (res != ESP_OK) break;
    }
    return res;
}

// ─────────────────────────────────────────────────────────────────────────────
// Single-snapshot handler — kept for compatibility.
// Returns 503 while scanning so the dashboard can detect it easily.
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t capture_handler(httpd_req_t *req) {
    if (QR_Reader_IsScanning()) {
        httpd_resp_set_status(req, "503 Busy");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, "scanning", 8);
    }

    SemaphoreHandle_t mtx = QR_Reader_GetMutex();
    if (xSemaphoreTake(mtx, pdMS_TO_TICKS(200)) != pdTRUE) {
        httpd_resp_set_status(req, "503 Busy");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, "scanning", 8);
    }

    camera_fb_t *fb = esp_camera_fb_get();
    xSemaphoreGive(mtx);

    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

// ─────────────────────────────────────────────────────────────────────────────
// Scan status handler — allows dashboard to poll for "scanning" vs "idle".
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t scan_status_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, QR_Reader_IsScanning() ? "scanning" : "idle", -1);
}

void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // ── Port 80: snapshot & status ───────────────────────────────────────────
    config.server_port = 80;
    httpd_uri_t capture_uri = {
        .uri      = "/capture",
        .method   = HTTP_GET,
        .handler  = capture_handler,
        .user_ctx = NULL
    };
    httpd_uri_t status_uri = {
        .uri      = "/scan_status",
        .method   = HTTP_GET,
        .handler  = scan_status_handler,
        .user_ctx = NULL
    };

    if (httpd_start(&cam_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(cam_httpd, &capture_uri);
        httpd_register_uri_handler(cam_httpd, &status_uri);
    }

    // ── Port 81: MJPEG stream ────────────────────────────────────────────────
    config.server_port += 1;
    config.ctrl_port   += 1;   // must also increment — two httpd servers can't share the same ctrl_port
    httpd_uri_t stream_uri = {
        .uri      = "/stream",
        .method   = HTTP_GET,
        .handler  = stream_handler,
        .user_ctx = NULL
    };

    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}

void setup() {
    UART_Slave_Init();
    QR_Reader_Init();
    Ultrasonic_Init();

    WiFi.mode(WIFI_STA);
    WiFi.begin(base_ssid, base_password);
    Serial.print("CAM_WIFI:connecting");

    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        String ipStr = WiFi.localIP().toString();
        Serial.println("CAM_IP:" + ipStr);
        Serial.println("Stream  : http://" + ipStr + ":81/stream");
        Serial.println("Snapshot: http://" + ipStr + "/capture");
        startCameraServer();
    } else {
        Serial.println("CAM_WIFI:failed");
    }
}

void loop() {
    // ── Receive SCAN_QR from base via UART ───────────────────────────────────
    if (UART_CheckForCommand("SCAN_QR")) {
        float dist = Ultrasonic_Read();
        UART_SendResult("DIST:" + String(dist, 2));
        QR_Reader_RequestScan();
    }

    // ── Poll scan task result ─────────────────────────────────────────────────
    if (QR_Reader_ResultReady()) {
        String color = QR_Reader_GetResult();

        if (color == "red" || color == "green" || color == "blue") {
            UART_SendResult("QR_OK:" + color);
        } else {
            UART_SendResult("QR_OK:none");
        }
    }

    delay(50);   // same yield as cam_try
}