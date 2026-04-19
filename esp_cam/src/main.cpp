// ==========================================
// ESP32-CAM: QR + Ultrasonic + UART + WiFi Camera Server
// ==========================================
#include <Arduino.h>
#include "qr_reader.h"
#include "UART_slave_cam.h"
#include "Ultrasonic.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_camera.h"

// Join the BASE's WiFi network as a client
const char* base_ssid = "RobotController";
const char* base_password = "robot1234";

httpd_handle_t cam_httpd = NULL;

// Serve a single JPEG snapshot
esp_err_t capture_handler(httpd_req_t *req) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    size_t out_len = 0;
    uint8_t * out_buf = NULL;
    if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &out_buf, &out_len);
        if (!jpeg_converted) {
            esp_camera_fb_return(fb);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
    } else {
        out_len = fb->len;
        out_buf = fb->buf;
    }

    esp_err_t res = httpd_resp_send(req, (const char *)out_buf, out_len);

    if (fb->format != PIXFORMAT_JPEG && out_buf) {
        free(out_buf);
    }
    esp_camera_fb_return(fb);
    return res;
}

void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = capture_handler,
        .user_ctx = NULL
    };

    if (httpd_start(&cam_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(cam_httpd, &capture_uri);
    }
}

void setup() {
    UART_Slave_Init();
    QR_Reader_Init();
    Ultrasonic_Init();

    // Connect to the base's WiFi network as a station
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
        // Send cam IP to base so dashboard knows where to fetch images
        Serial.println("CAM_IP:" + ipStr);
        Serial.println("Camera ready at http://" + ipStr + "/capture");
        startCameraServer();
    } else {
        Serial.println("CAM_WIFI:failed");
    }
}

void loop() {
    if (UART_CheckForCommand("SCAN_QR")) {
        // Step 1: Take 6 ultrasonic readings and send averaged distance
        float dist = Ultrasonic_Read();
        UART_SendResult("DIST:" + String(dist, 2));

        // Step 2: Scan for QR code
        unsigned long scanStart = millis();
        bool found = false;
        
        while (millis() - scanStart < 5000) { 
            String color = QR_Reader_Scan();
            if (color == "red" || color == "green" || color == "blue") {
                UART_SendResult("QR_OK:" + color);
                found = true;
                break;
            }
            delay(50);
        }
        
        if (!found) {
            UART_SendResult("QR_OK:none"); 
        }
    }
    
    QR_Reader_Scan();
    delay(10);
}