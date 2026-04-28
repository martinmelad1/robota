#include "Standalone_Server.h"
#include "Servo_Control.h"
#include "driver/gpio.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

static unsigned long last_uart_rx_high = 0;

const char arm_html[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>Arm Fallback Control</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }
  body { font-family: sans-serif; background: #0f1117; color: #e2e8f0; padding: 12px; user-select: none; -webkit-user-select: none; }
  .header { display: flex; align-items: center; justify-content: space-between; margin-bottom: 20px; }
  .header h1 { font-size: 18px; color: #a78bfa; }
  #conn { font-size: 12px; }
  .connected { color: #34d399; }
  .disconnected { color: #ef4444; }

  .estop {
    width: 100%; padding: 16px; background: #450a0a; border: 2px solid #ef4444;
    border-radius: 12px; color: #ef4444; font-size: 18px; font-weight: 700; letter-spacing: 2px;
    margin-bottom: 20px; cursor: pointer; touch-action: manipulation;
  }
  .estop:active { background: #ef4444; color: #fff; }

  .sec-label { font-size: 11px; color: #7c83a0; text-transform: uppercase; letter-spacing: 1px; margin-bottom: 10px; }

  .joint-row { display: grid; grid-template-columns: 40px 1fr 1fr; gap: 8px; align-items: stretch; margin-bottom: 10px; }
  .joint-label {
    font-size: 14px; font-weight: 700; color: #7c83a0; text-align: center;
    background: #1e2130; border-radius: 8px; padding: 10px 0;
    display: flex; align-items: center; justify-content: center;
  }
  .btn-arm {
    background: #0f2a1a; border: 2px solid #34d399; border-radius: 10px;
    color: #34d399; font-size: 24px; padding: 22px 6px;
    cursor: pointer; touch-action: none;
    display: flex; align-items: center; justify-content: center;
    flex-direction: column; gap: 4px; transition: background 0.08s, transform 0.08s;
  }
  .btn-arm .lbl { font-size: 11px; color: #34d399; }
  .btn-arm.active { background: #34d399; color: #000; transform: scale(0.95); }
  .btn-arm.active .lbl { color: #000; }

  .gripper-row { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; margin-top: 10px; }
  .btn-grip {
    padding: 20px 6px; border-radius: 12px; font-size: 26px; font-weight: 700;
    text-align: center; cursor: pointer; touch-action: manipulation; border: 2px solid;
    transition: all 0.12s; display: flex; flex-direction: column; align-items: center; gap: 6px;
  }
  .btn-grip .glbl { font-size: 11px; font-weight: 400; opacity: 0.8; }
  .grip-open { background: #0f1a2e; border-color: #60a5fa; color: #60a5fa; }
  .grip-open:active { background: #60a5fa; color: #000; }
  .grip-close { background: #1a0f2e; border-color: #a78bfa; color: #a78bfa; }
  .grip-close:active { background: #a78bfa; color: #000; }
  .grip-pick { background: #0f2a1a; border-color: #34d399; color: #34d399; }
  .grip-pick:active { background: #34d399; color: #000; }
  .divider { border: none; border-top: 1px solid #1e2130; margin: 16px 0; }
</style>
</head>
<body>
<div class="header">
  <h1>Arm Standalone Control</h1>
  <span id="conn" class="disconnected">● Connecting...</span>
</div>

<button class="estop" onpointerdown="emergencyStop()">&#9632; EMERGENCY STOP</button>

<div class="sec-label">Arm — press once to move, release to stop</div>
<div class="joint-row">
  <div class="joint-label">J1</div>
  <div class="btn-arm" id="j1up">&#x21BA;<span class="lbl">CCW</span></div>
  <div class="btn-arm" id="j1dn">&#x21BB;<span class="lbl">CW</span></div>
</div>
<div class="joint-row">
  <div class="joint-label">J2</div>
  <div class="btn-arm" id="j2up">&#x25B2;<span class="lbl">Up</span></div>
  <div class="btn-arm" id="j2dn">&#x25BC;<span class="lbl">Down</span></div>
</div>
<div class="joint-row">
  <div class="joint-label">J3</div>
  <div class="btn-arm" id="j3up">&#x25B2;<span class="lbl">Up</span></div>
  <div class="btn-arm" id="j3dn">&#x25BC;<span class="lbl">Down</span></div>
</div>
<div class="joint-row">
  <div class="joint-label" style="font-size:10px;">GRIP</div>
  <div class="btn-arm" id="gripup">&#x1F91C;<span class="lbl">Open</span></div>
  <div class="btn-arm" id="gripdn">&#x270A;<span class="lbl">Close</span></div>
</div>


<hr class="divider">

<div class="sec-label">Gripper — tap to activate</div>
<div class="gripper-row">
  <div class="btn-grip grip-open"  onpointerdown="send('GRIP_TAP_OPEN')">&#x1F91C;<span class="glbl">Open</span></div>
  <div class="btn-grip grip-close" onpointerdown="send('GRIP_TAP_CLOSE')">&#x270A;<span class="glbl">Close</span></div>
  <div class="btn-grip grip-pick"  onpointerdown="send('GRIP_TAP_PICK')">&#x1F4E6;<span class="glbl">Pick</span></div>
</div>

<script>
  let ws;
  const connEl = document.getElementById("conn");

  function connect() {
    ws = new WebSocket("ws://" + location.hostname + "/ws");
    ws.onopen = () => { connEl.textContent = "● Connected"; connEl.className = "connected"; };
    ws.onclose = () => { connEl.textContent = "● Disconn"; connEl.className = "disconnected"; setTimeout(connect, 2000); };
  }
  connect();

  function send(cmd) { if (ws && ws.readyState === WebSocket.OPEN) ws.send(cmd); }

  function emergencyStop() {
    send("ESTOP");
    document.body.style.background = "#450a0a";
    setTimeout(() => { document.body.style.background = "#0f1117"; }, 300);
  }

  function bindHoldButton(id, pressCmd, releaseCmd) {
    const el = document.getElementById(id);
    if (!el) return;
    el.addEventListener("pointerdown", e => {
      e.preventDefault(); el.setPointerCapture(e.pointerId);
      el.classList.add("active"); send(pressCmd);
    });
    el.addEventListener("pointerup", () => { el.classList.remove("active"); send(releaseCmd); });
    el.addEventListener("pointercancel", () => { el.classList.remove("active"); send(releaseCmd); });
  }

  bindHoldButton("j1up", "J1_UP",   "ARM_STOP");
  bindHoldButton("j1dn", "J1_DOWN", "ARM_STOP");
  bindHoldButton("j2up", "J2_UP",   "ARM_STOP");
  bindHoldButton("j2dn", "J2_DOWN", "ARM_STOP");
  bindHoldButton("j3up", "J3_UP",   "ARM_STOP");
  bindHoldButton("j3dn", "J3_DOWN", "ARM_STOP");
  bindHoldButton("gripup", "GRIP_OPEN", "ARM_STOP");
  bindHoldButton("gripdn", "GRIP_CLOSE", "ARM_STOP");

</script>
</body>
</html>)rawliteral";

static void processWsCommand(String cmd) {
  // If UART RX was high recently, the base is connected. IGNORE WIFI COMMANDS.
  if (millis() - last_uart_rx_high < 1000) {
    Serial.println("\n[ARM] UART Base detected. Ignoring Wi-Fi Command: " +
                   cmd);
    return;
  }

  ServoCommand sc;
  sc.joint_id = -1;
  sc.direction = 0;

  if (cmd == "J1_UP") {
    sc.joint_id = 1;
    sc.direction = 1;
  } else if (cmd == "J1_DOWN") {
    sc.joint_id = 1;
    sc.direction = -1;
  } else if (cmd == "J2_UP") {
    sc.joint_id = 2;
    sc.direction = 1;
  } else if (cmd == "J2_DOWN") {
    sc.joint_id = 2;
    sc.direction = -1;
  } else if (cmd == "J3_UP") {
    sc.joint_id = 3;
    sc.direction = -1;
  } else if (cmd == "J3_DOWN") {
    sc.joint_id = 3;
    sc.direction = 1;
  }

  else if (cmd == "ARM_STOP" || cmd == "ESTOP") {
    sc.joint_id = 0;
    sc.direction = 0;
  } else if (cmd == "GRIP_OPEN") {
    sc.joint_id = 5;
    sc.direction = 1;
  } else if (cmd == "GRIP_CLOSE") {
    sc.joint_id = 5;
    sc.direction = -1;
  } else if (cmd == "GRIP_TAP_OPEN") {
    sc.joint_id = 6;
    sc.direction = 1;
  } else if (cmd == "GRIP_TAP_CLOSE") {
    sc.joint_id = 6;
    sc.direction = -1;
  } else if (cmd == "GRIP_TAP_PICK") {
    sc.joint_id = 6;
    sc.direction = 2;
  }

  if (sc.joint_id != -1 && servoMailbox != NULL) {
    xQueueSend(servoMailbox, &sc, 0);
  }
}

static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    String msg = String((char *)data).substring(0, len);
    processWsCommand(msg);
  }
}

void Standalone_Server_Init() {
  // Explicitly pull down RX (pin 16). If the Master Base is powered and
  // connected, its TX line will drive this HIGH. If unplugged, it will drop
  // LOW.
  gpio_set_pull_mode((gpio_num_t)16, GPIO_PULLDOWN_ONLY);

  // Fallback Wi-Fi and Web server
  WiFi.softAP("ArmControl", "robot1234");

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "text/html", arm_html);
  });
  server.begin();

  Serial.println("Standalone Server Init Complete.");
  Serial.print("Connect to AP 'ArmControl' (pass: robot1234), IP: ");
  Serial.println(WiFi.softAPIP());
}

void Standalone_Server_Update() {
  // Read UART connection status
  if (gpio_get_level((gpio_num_t)16) == 1) {
    last_uart_rx_high = millis();
  }

  // Cleanup disconnected websocket clients
  ws.cleanupClients();
}
