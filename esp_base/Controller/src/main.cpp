#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

void consumerTask(void* param);
void broadcastTask(void* param);

// ── WiFi ──────────────────────────────────────────────────────
const char* ap_ssid     = "RobotController";
const char* ap_password = "robot1234";

AsyncWebServer server(80);
AsyncWebSocket  ws("/ws");

// ── Global state ──────────────────────────────────────────────
String current_command = "IDLE";
String current_mode    = "MANUAL";

// ── FreeRTOS Mailbox Queue ────────────────────────────────────
// Browser button press → WebSocket → xQueueSend → queue
// Consumer task        → xQueueReceive ← queue
// Queue holds 20 commands, each max 32 characters
#define Q_SIZE  20
#define Q_LEN   32
QueueHandle_t cmdQueue;

// ── Broadcast to browser ──────────────────────────────────────
void broadcastSensorData() {
  String json = "{";
  json += "\"cmd\":\""  + current_command + "\",";
  json += "\"mode\":\"" + current_mode    + "\",";
  json += "\"fl\":0,\"fr\":0,\"rl\":0,\"rr\":0,";
  json += "\"px\":0,\"py\":0,\"hdg\":0,";
  json += "\"j1\":90,\"j2\":90,\"j3\":90,\"j4\":90,";
  json += "\"grip\":1";
  json += "}";
  ws.textAll(json);
}

// ── WebSocket handler ─────────────────────────────────────────
// One job only: receive command from browser → put in queue
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    Serial.printf("Browser connected #%u\n", client->id());

  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("Browser disconnected #%u\n", client->id());

  } else if (type == WS_EVT_DATA) {
    char buf[Q_LEN] = {0};
    memcpy(buf, data, min(len, (size_t)(Q_LEN - 1)));

    // Put command into mailbox queue — returns instantly
    if (xQueueSend(cmdQueue, buf, 0) == pdTRUE) {
      Serial.println("Queued: " + String(buf));
    } else {
      Serial.println("Queue full — dropped: " + String(buf));
    }
  }
}

// ── Consumer task ─────────────────────────────────────────────
// This is where Base/Arm ESP-NOW sends will go later.
// For now it just reads from the queue and prints to terminal.
// Replace the Serial.println with esp_now_send() when ready.
void consumerTask(void* param) {
  char buf[Q_LEN];
  String lastCommand = "";

  for (;;) {
    if (xQueueReceive(cmdQueue, buf, portMAX_DELAY) == pdTRUE) {
      String cmd = String(buf);

      // Ignore duplicate commands
      if (cmd == lastCommand) {
        continue;
      }
      lastCommand = cmd;

      // Update state
      current_command = cmd;

      if (cmd == "MODE_MANUAL")    current_mode = "MANUAL";
      else if (cmd == "MODE_AUTO") current_mode = "AUTONOMOUS";
      else if (cmd == "MODE_PICK") current_mode = "AUTO PICK";

      Serial.println("CMD OUT → " + cmd);

      broadcastSensorData();
    }
  }
}

// ── Broadcast task ────────────────────────────────────────────
// Keeps dashboard alive every 50ms even when no commands sent
void broadcastTask(void* param) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(50));
    broadcastSensorData();
    ws.cleanupClients();
  }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  // Create queue first before anything uses it
  cmdQueue = xQueueCreate(Q_SIZE, Q_LEN);
  if (!cmdQueue) {
    Serial.println("Queue failed!"); while(1);
  }
  Serial.println("Queue ready");

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("Connect to   : "); Serial.println(ap_ssid);
  Serial.print("Dashboard    : http://"); Serial.println(WiFi.softAPIP());
  Serial.println("Control      : http://192.168.4.1/control");

  // LittleFS — serves dashboard.html and control.html
  if (!LittleFS.begin()) {
    Serial.println("LittleFS failed — run Upload Filesystem Image");
  }

  // WebSocket + HTTP server
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.serveStatic("/", LittleFS, "/").setDefaultFile("dashboard.html");
  server.on("/control", HTTP_GET, [](AsyncWebServerRequest* req){
    req->send(LittleFS, "/control.html", "text/html");
  });
  server.begin();
  Serial.println("Server ready — waiting for browser...");

  // Consumer task on Core 1 — reads queue, will send ESP-NOW later
  xTaskCreatePinnedToCore(consumerTask, "CMD", 4096, NULL, 2, NULL, 1);

  // Broadcast task on Core 0 — keeps dashboard updated
  xTaskCreatePinnedToCore(broadcastTask, "BC", 3072, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}