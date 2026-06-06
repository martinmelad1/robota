#include "StateMachine.h"
#include "WorldState.h"
#include "UART_Master.h"
#include "PID_Control.h"
#include "Odometry.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WiFi.h>

const char *ap_ssid = "RobotController";
const char *ap_password = "robot1234";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

MasterStateMachine robotBrain;
QueueHandle_t guiMailbox; // Now holds StringMessage
QueueHandle_t pidMailbox;
QueueHandle_t armMailbox;

TaskHandle_t BrainTask;
TaskHandle_t WebBroadcastTask;

// ── TELEMETRY BROADCAST ──────────────────────────────────────
void broadcastStateData()
{
  String json = robotBrain.getTelemetryJSON();
  ws.textAll(json);
}

// ── WEBSOCKET HANDLER ─────────────────────────────────────────
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_CONNECT)
  {
    Serial.printf("Client #%u connected\n", client->id());
  }
  else if (type == WS_EVT_DISCONNECT)
  {
    Serial.printf("Client #%u disconnected\n", client->id());
  }
  else if (type == WS_EVT_DATA)
  {
    String msg = String((char *)data).substring(0, len);
    Serial.println("RCV CMD: " + msg);

    StringMessage sc;
    strncpy(sc.data, msg.c_str(), sizeof(sc.data) - 1);
    sc.data[sizeof(sc.data) - 1] = '\0';

    // Push raw string packet to State Machine's mailbox for translation
    if (xQueueSend(guiMailbox, &sc, 0) != pdPASS)
    {
      Serial.println("Warning: guiMailbox full! Dropping WS command to prevent server lockup.");
    }
  }
}

// FreeRTOS TASKS

void setupTasks()
{
  // Odometry runs at 100 Hz on Core 1 — must start before Brain
  xTaskCreatePinnedToCore(
      [](void *pvParameters)
      {
        Odometry_Init();      // calibrates IMU, ~1 s still
        Odometry_ResetPose(); // zero the world-frame origin
        while (true)
        {
          Odometry_Update();
          vTaskDelay(10 / portTICK_PERIOD_MS); // 100 Hz
        }
      },
      "OdoTask", 4096, NULL, 3 /*BUG3 FIX: higher priority than BrainTask*/, NULL, 1);

  // Master State Machine logic runs continuously
  xTaskCreatePinnedToCore(
      [](void *pvParameters)
      {
        robotBrain.init();
        while (true)
        {
          robotBrain.update();
          vTaskDelay(20 / portTICK_PERIOD_MS); // 50 Hz Brain Tick
        }
      },
      "BrainTask", 8192, NULL, 2, &BrainTask, 1);

  // Simple telemetry broadcast task to keep GUI alive
  // Later we can implement listening to UART/Mailboxes to relay external data
  xTaskCreatePinnedToCore(
      [](void *pvParameters)
      {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        while (true)
        {
          broadcastStateData();
          vTaskDelayUntil(&xLastWakeTime,
                          100 / portTICK_PERIOD_MS); // 10Hz Broadcast
        }
      },
      "WebBroadcastTask", 4096, NULL, 1, &WebBroadcastTask, 0);

  // Start UART Master Tasks
  xTaskCreatePinnedToCore(UART_Cam_Task, "UART_Cam", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(UART_Arm_Task, "UART_Arm", 4096, NULL, 1, NULL, 1);

  // Start PID Task
  PID_StartTask();
}

// ── SETUP ─────────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);

  // Initialize ESP-IDF UART interfaces
  UART_Master_Init();
  PID_Init();

  guiMailbox = xQueueCreate(10, sizeof(StringMessage));
  pidMailbox = xQueueCreate(1, sizeof(ChassisMotion));
  armMailbox = xQueueCreate(1, sizeof(ArmMotion));

  if (!LittleFS.begin())
  {
    Serial.println("LittleFS mount failed");
    return;
  }

  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("Open browser at: http://");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.serveStatic("/", LittleFS, "/").setDefaultFile("dashboard.html");
  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *req)
            { req->send(LittleFS, "/control.html", "text/html"); });
  server.begin();
  Serial.println("Server started");

  setupTasks();
}

// LOOP
void loop()
{
  // FreeRTOS tasks handle the routing.
  // We just clean up WebSockets clients .
  ws.cleanupClients();

  // Simple serial testing interface for hardware testing
  UART_Master_ProcessSerial();

  delay(50);
}