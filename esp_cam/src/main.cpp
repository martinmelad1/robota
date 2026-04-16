// WEBCAM_DEBUG modes:
//   0 = Production (QR scanning + UART + WiFi camera server)
//   1 = Lens Focus mode (just shows camera image on web, no QR scanning)
//   2 = QR Debug mode (shows camera image + QR decode result on web page, own AP)
#define WEBCAM_DEBUG 0

#if WEBCAM_DEBUG == 0
// ==========================================
// PRODUCTION MODE: QR + UART + Camera Server
// ==========================================
#include <Arduino.h>
#include "qr_reader.h"
#include "UART_slave_cam.h"
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

#elif WEBCAM_DEBUG == 2
// ==========================================
// QR DEBUG MODE: Web page shows image + result (own AP)
// ==========================================
#include <Arduino.h>
#include "qr_reader.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_camera.h"

const char* ssid = "ROBOT_EYE";
const char* password = "";

httpd_handle_t server_httpd = NULL;

static volatile bool qrFound = false;
static char qrPayload[256] = "";
static unsigned long qrFoundTime = 0;

esp_err_t capture_handler(httpd_req_t *req) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    
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

esp_err_t status_handler(httpd_req_t *req) {
    char json[320];
    if (qrFound) {
        snprintf(json, sizeof(json), "{\"found\":true,\"payload\":\"%s\",\"age\":%lu}", qrPayload, millis() - qrFoundTime);
    } else {
        snprintf(json, sizeof(json), "{\"found\":false,\"payload\":\"\",\"age\":0}");
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

esp_err_t index_handler(httpd_req_t *req) {
    const char* html = 
        "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>"
        "body{background:#111;color:#fff;font-family:monospace;text-align:center;margin:0;padding:10px;}"
        "img{width:100%;max-width:640px;border:2px solid #333;border-radius:8px;}"
        "#status{font-size:24px;padding:15px;margin:10px;border-radius:8px;}"
        ".found{background:#0a5;color:#fff;}"
        ".scanning{background:#333;color:#aaa;}"
        "</style></head><body>"
        "<h2>ESP32-CAM QR Debug</h2>"
        "<img id='cam' src='/capture'>"
        "<div id='status' class='scanning'>Scanning...</div>"
        "<script>"
        "setInterval(()=>{"
          "document.getElementById('cam').src='/capture?'+Math.random();"
          "fetch('/status').then(r=>r.json()).then(d=>{"
            "var el=document.getElementById('status');"
            "if(d.found){"
              "el.className='found';"
              "el.innerHTML='QR DETECTED: <b>'+d.payload+'</b>';"
            "}else{"
              "el.className='scanning';"
              "el.innerHTML='Scanning... no QR code found yet';"
            "}"
          "});"
        "},800);"
        "</script></body></html>";
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

void startServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
    httpd_uri_t capture_uri = { .uri = "/capture", .method = HTTP_GET, .handler = capture_handler, .user_ctx = NULL };
    httpd_uri_t status_uri = { .uri = "/status", .method = HTTP_GET, .handler = status_handler, .user_ctx = NULL };

    if (httpd_start(&server_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(server_httpd, &index_uri);
        httpd_register_uri_handler(server_httpd, &capture_uri);
        httpd_register_uri_handler(server_httpd, &status_uri);
    }
}

void setup() {
    Serial.begin(115200);
    QR_Reader_Init();

    WiFi.softAP(ssid, password);
    Serial.print("QR Debug Page: http://");
    Serial.print(WiFi.softAPIP());
    Serial.println("/");

    startServer();
}

void loop() {
    String result = QR_Reader_Scan();
    if (result.length() > 0) {
        qrFound = true;
        strncpy(qrPayload, result.c_str(), sizeof(qrPayload) - 1);
        qrPayload[sizeof(qrPayload) - 1] = '\0';
        qrFoundTime = millis();
        Serial.println(">>> QR DETECTED: " + result);
    }
    delay(50);
}

#else
// ==========================================
// LENS FOCUS MODE: Just shows camera image
// ==========================================
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const char* ssid = "ROBOT_EYE";
const char* password = "";

httpd_handle_t stream_httpd = NULL;

esp_err_t capture_handler(httpd_req_t *req){
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    
    size_t out_len = 0;
    uint8_t * out_buf = NULL;
    if(fb->format != PIXFORMAT_JPEG){
        bool jpeg_converted = frame2jpg(fb, 80, &out_buf, &out_len);
        if(!jpeg_converted){
            esp_camera_fb_return(fb);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
    } else {
        out_len = fb->len;
        out_buf = fb->buf;
    }

    esp_err_t res = httpd_resp_send(req, (const char *)out_buf, out_len);

    if(fb->format != PIXFORMAT_JPEG && out_buf){
        free(out_buf);
    }
    esp_camera_fb_return(fb);
    return res;
}

esp_err_t index_handler(httpd_req_t *req){
    const char* html = "<html><body style='text-align:center; background:#000;'><h2 style='color:#fff;'>Lens Focus (Auto-refresh)</h2>"
                       "<img id='cam' src='/capture' style='width:100%; max-width:640px;'>"
                       "<script>setInterval(() => { document.getElementById('cam').src = '/capture?' + Math.random(); }, 500);</script>"
                       "</body></html>";
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
    httpd_uri_t capture_uri = { .uri = "/capture", .method = HTTP_GET, .handler = capture_handler, .user_ctx = NULL };
    
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &index_uri);
        httpd_register_uri_handler(stream_httpd, &capture_uri);
    }
}

void setup() {
    Serial.begin(115200);
    
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = 5;  config.pin_d1 = 18; config.pin_d2 = 19; config.pin_d3 = 21;
    config.pin_d4 = 36; config.pin_d5 = 39; config.pin_d6 = 34; config.pin_d7 = 35;
    config.pin_xclk = 0;  config.pin_pclk = 22;
    config.pin_vsync = 25; config.pin_href = 23;
    config.pin_sccb_sda = 26; config.pin_sccb_scl = 27;
    config.pin_pwdn = 32;  config.pin_reset = -1;
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_VGA; 
    config.jpeg_quality = 10;
    config.fb_count = 1;

    esp_camera_init(&config);

    WiFi.softAP(ssid, password);
    Serial.print("Webcam: http://");
    Serial.println(WiFi.softAPIP());

    startCameraServer();
}

void loop() {
    delay(1000);
}

#endif