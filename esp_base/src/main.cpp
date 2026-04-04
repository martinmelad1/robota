#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "WorldState.h"
#include "StateMachine.h"

const char* ap_ssid     = "RobotController";
const char* ap_password = "robot1234";

AsyncWebServer server(80);
AsyncWebSocket  ws("/ws");

MasterStateMachine robotBrain;
QueueHandle_t guiMailbox;
QueueHandle_t pidMailbox;
QueueHandle_t armMailbox;
TaskHandle_t BrainTask;
// ── PIN DEFINITIONS ───────────────────────────────────────────
#define MTR_FL_PWM   2
#define MTR_FL_IN1   4
#define MTR_FL_IN2   5
#define MTR_FR_PWM   15
#define MTR_FR_IN1   16
#define MTR_FR_IN2   17
#define MTR_RL_PWM   18
#define MTR_RL_IN1   19
#define MTR_RL_IN2   21
#define MTR_RR_PWM   22
#define MTR_RR_IN1   23
#define MTR_RR_IN2   25

#define ENC_FL_A  26
#define ENC_FL_B  27
#define ENC_FR_A  32
#define ENC_FR_B  33
#define ENC_RL_A  34
#define ENC_RL_B  35
#define ENC_RR_A  36
#define ENC_RR_B  39

#define IMU_SDA  21
#define IMU_SCL  22

#define SERVO_J1_PIN  12
#define SERVO_J2_PIN  13
#define SERVO_J3_PIN  14
#define SERVO_J4_PIN  27
#define SERVO_J5_PIN  26

// ── SERVOS ────────────────────────────────────────────────────
Servo servoJ1, servoJ2, servoJ3, servoJ4, servoJ5;
int posJ1 = 90, posJ2 = 90, posJ3 = 90, posJ4 = 90;
bool gripperOpen = true;
#define SERVO_STEP        2
#define GRIPPER_OPEN_DEG  30
#define GRIPPER_CLOSE_DEG 120

// ── ENCODERS ─────────────────────────────────────────────────
volatile long encFL = 0, encFR = 0, encRL = 0, encRR = 0;
#define PULSES_PER_REV   44
#define WHEEL_DIAMETER_M 0.1
#define METERS_PER_PULSE (3.14159 * WHEEL_DIAMETER_M / PULSES_PER_REV)

void IRAM_ATTR isrFL() { encFL++; }
void IRAM_ATTR isrFR() { encFR++; }
void IRAM_ATTR isrRL() { encRL++; }
void IRAM_ATTR isrRR() { encRR++; }

// ── IMU ───────────────────────────────────────────────────────
#define MPU6050_ADDR 0x68
float heading      = 0;
float gyroZ_offset = 0;
unsigned long lastIMURead = 0;

void broadcastSensorData();
float readGyroZ() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  // ✅ FIX 1: false = non-blocking, stops I2C from stalling the loop
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return (float)raw / 131.0;
}

void updateIMU() {
  unsigned long now = millis();
  float dt = (now - lastIMURead) / 1000.0;
  lastIMURead = now;
  float gz = readGyroZ() - gyroZ_offset;
  if (abs(gz) > 0.5) heading += gz * dt;
  if (heading >  360) heading -= 360;
  if (heading <  0  ) heading += 360;
}

void calibrateIMU() {
  Serial.println("Calibrating IMU — keep robot still...");
  float sum = 0;
  for (int i = 0; i < 200; i++) { sum += readGyroZ(); delay(10); }
  gyroZ_offset = sum / 200.0;
  Serial.println("IMU calibrated. Offset: " + String(gyroZ_offset));
}

// ── ODOMETRY ─────────────────────────────────────────────────
float pos_x = 0, pos_y = 0;
long  lastEncFL = 0, lastEncFR = 0;

void updateOdometry() {
  long dFL = encFL - lastEncFL;
  long dFR = encFR - lastEncFR;
  lastEncFL = encFL;
  lastEncFR = encFR;
  float distMeters = ((dFL + dFR) / 2.0) * METERS_PER_PULSE;
  float headRad = heading * 3.14159 / 180.0;
  pos_x += distMeters * cos(headRad);
  pos_y += distMeters * sin(headRad);
}

// ── MOTOR CONTROL ─────────────────────────────────────────────
int driveSpeed = 180;

void setMotor(int pwmPin, int in1, int in2, int speed, bool forward) {
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW  : HIGH);
  analogWrite(pwmPin, speed);
}

void stopAllMotors() {
  analogWrite(MTR_FL_PWM, 0); analogWrite(MTR_FR_PWM, 0);
  analogWrite(MTR_RL_PWM, 0); analogWrite(MTR_RR_PWM, 0);
}

void driveFwd() {
  setMotor(MTR_FL_PWM, MTR_FL_IN1, MTR_FL_IN2, driveSpeed, true);
  setMotor(MTR_FR_PWM, MTR_FR_IN1, MTR_FR_IN2, driveSpeed, true);
  setMotor(MTR_RL_PWM, MTR_RL_IN1, MTR_RL_IN2, driveSpeed, true);
  setMotor(MTR_RR_PWM, MTR_RR_IN1, MTR_RR_IN2, driveSpeed, true);
}
void driveBwd() {
  setMotor(MTR_FL_PWM, MTR_FL_IN1, MTR_FL_IN2, driveSpeed, false);
  setMotor(MTR_FR_PWM, MTR_FR_IN1, MTR_FR_IN2, driveSpeed, false);
  setMotor(MTR_RL_PWM, MTR_RL_IN1, MTR_RL_IN2, driveSpeed, false);
  setMotor(MTR_RR_PWM, MTR_RR_IN1, MTR_RR_IN2, driveSpeed, false);
}
void strafeLeft() {
  setMotor(MTR_FL_PWM, MTR_FL_IN1, MTR_FL_IN2, driveSpeed, false);
  setMotor(MTR_FR_PWM, MTR_FR_IN1, MTR_FR_IN2, driveSpeed, true);
  setMotor(MTR_RL_PWM, MTR_RL_IN1, MTR_RL_IN2, driveSpeed, true);
  setMotor(MTR_RR_PWM, MTR_RR_IN1, MTR_RR_IN2, driveSpeed, false);
}
void strafeRight() {
  setMotor(MTR_FL_PWM, MTR_FL_IN1, MTR_FL_IN2, driveSpeed, true);
  setMotor(MTR_FR_PWM, MTR_FR_IN1, MTR_FR_IN2, driveSpeed, false);
  setMotor(MTR_RL_PWM, MTR_RL_IN1, MTR_RL_IN2, driveSpeed, false);
  setMotor(MTR_RR_PWM, MTR_RR_IN1, MTR_RR_IN2, driveSpeed, true);
}
void rotateLeft() {
  setMotor(MTR_FL_PWM, MTR_FL_IN1, MTR_FL_IN2, driveSpeed, false);
  setMotor(MTR_FR_PWM, MTR_FR_IN1, MTR_FR_IN2, driveSpeed, true);
  setMotor(MTR_RL_PWM, MTR_RL_IN1, MTR_RL_IN2, driveSpeed, false);
  setMotor(MTR_RR_PWM, MTR_RR_IN1, MTR_RR_IN2, driveSpeed, true);
}
void rotateRight() {
  setMotor(MTR_FL_PWM, MTR_FL_IN1, MTR_FL_IN2, driveSpeed, true);
  setMotor(MTR_FR_PWM, MTR_FR_IN1, MTR_FR_IN2, driveSpeed, false);
  setMotor(MTR_RL_PWM, MTR_RL_IN1, MTR_RL_IN2, driveSpeed, true);
  setMotor(MTR_RR_PWM, MTR_RR_IN1, MTR_RR_IN2, driveSpeed, false);
}
void diagFwdLeft() {
  analogWrite(MTR_FL_PWM, 0);
  setMotor(MTR_FR_PWM, MTR_FR_IN1, MTR_FR_IN2, driveSpeed, true);
  setMotor(MTR_RL_PWM, MTR_RL_IN1, MTR_RL_IN2, driveSpeed, true);
  analogWrite(MTR_RR_PWM, 0);
}
void diagFwdRight() {
  setMotor(MTR_FL_PWM, MTR_FL_IN1, MTR_FL_IN2, driveSpeed, true);
  analogWrite(MTR_FR_PWM, 0); analogWrite(MTR_RL_PWM, 0);
  setMotor(MTR_RR_PWM, MTR_RR_IN1, MTR_RR_IN2, driveSpeed, true);
}
void diagBwdLeft() {
  setMotor(MTR_FL_PWM, MTR_FL_IN1, MTR_FL_IN2, driveSpeed, false);
  analogWrite(MTR_FR_PWM, 0); analogWrite(MTR_RL_PWM, 0);
  setMotor(MTR_RR_PWM, MTR_RR_IN1, MTR_RR_IN2, driveSpeed, false);
}
void diagBwdRight() {
  analogWrite(MTR_FL_PWM, 0);
  setMotor(MTR_FR_PWM, MTR_FR_IN1, MTR_FR_IN2, driveSpeed, false);
  setMotor(MTR_RL_PWM, MTR_RL_IN1, MTR_RL_IN2, driveSpeed, false);
  analogWrite(MTR_RR_PWM, 0);
}

// ── ARM ───────────────────────────────────────────────────────
void moveJoint(Servo& servo, int& pos, int step) {
  pos = constrain(pos + step, 0, 180);
  servo.write(pos);
}
void openGripper()  { gripperOpen = true;  servoJ5.write(GRIPPER_OPEN_DEG); }
void closeGripper() { gripperOpen = false; servoJ5.write(GRIPPER_CLOSE_DEG); }

// ── STATE ─────────────────────────────────────────────────────
String current_command = "IDLE";
String current_mode    = "MANUAL";

// ── WEBSOCKET HANDLER ─────────────────────────────────────────
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    Serial.printf("Client #%u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("Client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    String msg = String((char*)data).substring(0, len);
    Serial.println("CMD: " + msg);
    current_command = msg;

    if      (msg == "ESTOP")       { stopAllMotors(); current_command = "EMERGENCY STOP"; }
    else if (msg == "MODE_MANUAL") { current_mode = "MANUAL"; }
    else if (msg == "MODE_AUTO")   { current_mode = "AUTONOMOUS"; }
    else if (msg == "FWD")         { driveFwd(); }
    else if (msg == "BWD")         { driveBwd(); }
    else if (msg == "LEFT")        { strafeLeft(); }
    else if (msg == "RIGHT")       { strafeRight(); }
    else if (msg == "FWD_LEFT")    { diagFwdLeft(); }
    else if (msg == "FWD_RIGHT")   { diagFwdRight(); }
    else if (msg == "BWD_LEFT")    { diagBwdLeft(); }
    else if (msg == "BWD_RIGHT")   { diagBwdRight(); }
    else if (msg == "ROT_L")       { rotateLeft(); }
    else if (msg == "ROT_R")       { rotateRight(); }
    else if (msg == "STOP")        { stopAllMotors(); current_command = "IDLE"; }
    else if (msg == "J1_UP")       { moveJoint(servoJ1, posJ1,  SERVO_STEP); }
    else if (msg == "J1_DOWN")     { moveJoint(servoJ1, posJ1, -SERVO_STEP); }
    else if (msg == "J2_UP")       { moveJoint(servoJ2, posJ2,  SERVO_STEP); }
    else if (msg == "J2_DOWN")     { moveJoint(servoJ2, posJ2, -SERVO_STEP); }
    else if (msg == "J3_UP")       { moveJoint(servoJ3, posJ3,  SERVO_STEP); }
    else if (msg == "J3_DOWN")     { moveJoint(servoJ3, posJ3, -SERVO_STEP); }
    else if (msg == "J4_UP")       { moveJoint(servoJ4, posJ4,  SERVO_STEP); }
    else if (msg == "J4_DOWN")     { moveJoint(servoJ4, posJ4, -SERVO_STEP); }
    else if (msg == "ARM_STOP")    { current_command = "IDLE"; }
    else if (msg == "GRIP_OPEN")   { openGripper(); }
    else if (msg == "GRIP_CLOSE")  { closeGripper(); }

    // ✅ FIX 2: broadcast immediately on every command so the dashboard
    //    updates right away, not after waiting up to 100ms for the timer
    updateOdometry();
    broadcastSensorData();
  }
}

// ── BROADCAST ────────────────────────────────────────────────
void broadcastSensorData() {
  static long lastFL = 0, lastFR = 0, lastRL = 0, lastRR = 0;
  float speedFL = (encFL - lastFL) * METERS_PER_PULSE * 20.0; // 20Hz now
  float speedFR = (encFR - lastFR) * METERS_PER_PULSE * 20.0;
  float speedRL = (encRL - lastRL) * METERS_PER_PULSE * 20.0;
  float speedRR = (encRR - lastRR) * METERS_PER_PULSE * 20.0;
  lastFL = encFL; lastFR = encFR; lastRL = encRL; lastRR = encRR;

  String json = "{";
  json += "\"cmd\":\""  + current_command    + "\",";
  json += "\"mode\":\"" + current_mode       + "\",";
  json += "\"fl\":"     + String(speedFL, 3) + ",";
  json += "\"fr\":"     + String(speedFR, 3) + ",";
  json += "\"rl\":"     + String(speedRL, 3) + ",";
  json += "\"rr\":"     + String(speedRR, 3) + ",";
  json += "\"px\":"     + String(pos_x, 3)   + ",";
  json += "\"py\":"     + String(pos_y, 3)   + ",";
  json += "\"hdg\":"    + String(heading, 1) + ",";
  json += "\"j1\":"     + String(posJ1)      + ",";
  json += "\"j2\":"     + String(posJ2)      + ",";
  json += "\"j3\":"     + String(posJ3)      + ",";
  json += "\"j4\":"     + String(posJ4)      + ",";
  json += "\"grip\":"   + String(gripperOpen ? 1 : 0);
  json += "}";

  ws.textAll(json);
}

// ── SETUP ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  guiMailbox = xQueueCreate(1, sizeof(GUIPacket));
  pidMailbox = xQueueCreate(1, sizeof(ChassisMotion));
  armMailbox = xQueueCreate(1, sizeof(ArmMotion));

  pinMode(MTR_FL_IN1, OUTPUT); pinMode(MTR_FL_IN2, OUTPUT);
  pinMode(MTR_FR_IN1, OUTPUT); pinMode(MTR_FR_IN2, OUTPUT);
  pinMode(MTR_RL_IN1, OUTPUT); pinMode(MTR_RL_IN2, OUTPUT);
  pinMode(MTR_RR_IN1, OUTPUT); pinMode(MTR_RR_IN2, OUTPUT);
  stopAllMotors();

  pinMode(ENC_FL_A, INPUT_PULLUP); pinMode(ENC_FR_A, INPUT_PULLUP);
  pinMode(ENC_RL_A, INPUT_PULLUP); pinMode(ENC_RR_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), isrFL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), isrFR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RL_A), isrRL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_A), isrRR, RISING);

  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);
  calibrateIMU();
  lastIMURead = millis();

  servoJ1.attach(SERVO_J1_PIN); servoJ2.attach(SERVO_J2_PIN);
  servoJ3.attach(SERVO_J3_PIN); servoJ4.attach(SERVO_J4_PIN);
  servoJ5.attach(SERVO_J5_PIN);
  servoJ1.write(posJ1); servoJ2.write(posJ2);
  servoJ3.write(posJ3); servoJ4.write(posJ4);
  openGripper();

  if (!LittleFS.begin()) { Serial.println("LittleFS mount failed"); return; }

  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("Open browser at: http://"); Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.serveStatic("/", LittleFS, "/").setDefaultFile("dashboard.html");
  server.on("/control", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(LittleFS, "/control.html", "text/html");
  });
  server.begin();
  Serial.println("Server started");
}

// ── LOOP ──────────────────────────────────────────────────────
void loop() {
  ws.cleanupClients();

  unsigned long now = millis();

  // ✅ FIX 3: IMU runs every 20ms only — NOT every loop iteration
  //    Before this, readGyroZ() (blocking I2C) ran thousands of times/sec
  static unsigned long lastIMU = 0;
  if (now - lastIMU >= 20) {
    lastIMU = now;
    updateIMU();
  }

  // ✅ FIX 4: broadcast every 50ms (20Hz) instead of 100ms (10Hz)
  //    Halves the maximum dashboard update delay
  static unsigned long lastBroadcast = 0;
  if (now - lastBroadcast >= 50) {
    lastBroadcast = now;
    updateOdometry();
    broadcastSensorData();
  }
}