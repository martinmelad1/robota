// IMU.cpp — MPU6050 heading & acceleration, esp_arm
// Raw I2C only — no external library needed.
// Runs as a 100 Hz FreeRTOS task on Core 0.

#include "IMU.h"
#include <Wire.h>
#include <math.h>

// ── MPU6050 register addresses ────────────────────────────────
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H  0x43
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A

// Scale factors for default ±2g accel, ±250°/s gyro
static constexpr float ACCEL_SCALE = 16384.0f;   // LSB/g
static constexpr float GYRO_SCALE  = 131.0f;     // LSB/(°/s)
static constexpr float G_MS2       = 9.80665f;   // m/s²
static constexpr float DEG2RAD     = float(M_PI) / 180.0f;
static constexpr float RAD2DEG     = 180.0f / float(M_PI);

// ── Module state ──────────────────────────────────────────────
static portMUX_TYPE  g_mux      = portMUX_INITIALIZER_UNLOCKED;
static float         g_yaw      = 0.0f;   // degrees
static float         g_accelX   = 0.0f;   // m/s²
static float         g_accelY   = 0.0f;   // m/s²
static bool          g_ready    = false;

// Gyro bias (calibrated at startup, robot must be still)
static float g_gyroBiasX = 0.0f;
static float g_gyroBiasY = 0.0f;
static float g_gyroBiasZ = 0.0f;

// ── Low-level I2C helpers ─────────────────────────────────────

static void _writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// Read `count` bytes starting at `reg` into `buf`
static bool _readRegs(uint8_t reg, uint8_t *buf, uint8_t count) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)IMU_I2C_ADDR, count);
    for (uint8_t i = 0; i < count && Wire.available(); i++)
        buf[i] = Wire.read();
    return true;
}

static int16_t _int16(uint8_t hi, uint8_t lo) {
    return (int16_t)((hi << 8) | lo);
}

// ── Calibration ───────────────────────────────────────────────
// Average 500 gyro readings with robot still to get bias offset.
static void _calibrateGyro() {
    Serial.println("[IMU] Calibrating — keep robot still for 1 s...");
    float sx = 0, sy = 0, sz = 0;
    const int N = 500;
    for (int i = 0; i < N; i++) {
        uint8_t raw[6];
        if (_readRegs(REG_GYRO_XOUT_H, raw, 6)) {
            sx += _int16(raw[0], raw[1]) / GYRO_SCALE;
            sy += _int16(raw[2], raw[3]) / GYRO_SCALE;
            sz += _int16(raw[4], raw[5]) / GYRO_SCALE;
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
    g_gyroBiasX = sx / N;
    g_gyroBiasY = sy / N;
    g_gyroBiasZ = sz / N;
    Serial.printf("[IMU] Gyro bias: X=%.4f Y=%.4f Z=%.4f °/s\n",
                  g_gyroBiasX, g_gyroBiasY, g_gyroBiasZ);
}

// ── Init ──────────────────────────────────────────────────────
void IMU_Init() {
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);  // 400 kHz fast mode

    // Wake the chip (clear sleep bit)
    _writeReg(REG_PWR_MGMT_1, 0x00);
    delay(100);

    // Sample rate = 1 kHz / (1 + SMPLRT_DIV) — set to 100 Hz
    _writeReg(REG_SMPLRT_DIV, 9);

    // DLPF ~44 Hz (reduces vibration noise on chassis)
    _writeReg(REG_CONFIG, 0x03);

    // Gyro ±250 °/s, Accel ±2 g (defaults, scale factors above)
    _writeReg(REG_GYRO_CONFIG,  0x00);
    _writeReg(REG_ACCEL_CONFIG, 0x00);

    // Verify connection
    Wire.beginTransmission(IMU_I2C_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[IMU] ERROR: MPU6050 not found! Check SDA/SCL wiring.");
        return;
    }
    Serial.println("[IMU] MPU6050 found.");

    // Calibrate (robot must be still during init)
    _calibrateGyro();
    g_ready = true;
    Serial.println("[IMU] Ready.");
}

// ── 100 Hz update task ────────────────────────────────────────
static void _imuTask(void *arg) {
    // Complementary filter state
    float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
    unsigned long lastUs = micros();

    while (true) {
        unsigned long nowUs = micros();
        float dt = (nowUs - lastUs) * 1e-6f;
        lastUs = nowUs;
        if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

        uint8_t raw[14];
        if (!_readRegs(REG_ACCEL_XOUT_H, raw, 14)) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        // Raw sensor values
        float ax = _int16(raw[0],  raw[1])  / ACCEL_SCALE;   // g
        float ay = _int16(raw[2],  raw[3])  / ACCEL_SCALE;
        float az = _int16(raw[4],  raw[5])  / ACCEL_SCALE;
        // raw[6..7] = temperature, skip
        float gx = _int16(raw[8],  raw[9])  / GYRO_SCALE - g_gyroBiasX;  // °/s
        float gy = _int16(raw[10], raw[11]) / GYRO_SCALE - g_gyroBiasY;
        float gz = _int16(raw[12], raw[13]) / GYRO_SCALE - g_gyroBiasZ;

        // Accel-derived pitch and roll (degrees)
        float accelPitch = atan2f(ay, sqrtf(ax*ax + az*az)) * RAD2DEG;
        float accelRoll  = atan2f(-ax, az) * RAD2DEG;

        // Complementary filter — fuse gyro integration with accel tilt
        pitch = IMU_COMP_ALPHA * (pitch + gx * dt) + (1.0f - IMU_COMP_ALPHA) * accelPitch;
        roll  = IMU_COMP_ALPHA * (roll  + gy * dt) + (1.0f - IMU_COMP_ALPHA) * accelRoll;

        // Yaw = pure gyro integration (no accel correction — accel can't sense yaw)
        // This drifts slowly; acceptable for short autonomous runs.
        yaw += gz * dt;

        // Wrap yaw to ±180°
        while (yaw >  180.0f) yaw -= 360.0f;
        while (yaw < -180.0f) yaw += 360.0f;

        // World-frame acceleration (remove gravity component using tilt angles)
        // Simple first-order approximation: subtract gravity projection
        float sinPitch = sinf(pitch * DEG2RAD);
        float sinRoll  = sinf(roll  * DEG2RAD);
        float ax_world = (ax - sinRoll)  * G_MS2;
        float ay_world = (ay - sinPitch) * G_MS2;

        // Write to shared state under mutex
        portENTER_CRITICAL(&g_mux);
        g_yaw    = yaw;
        g_accelX = ax_world;
        g_accelY = ay_world;
        portEXIT_CRITICAL(&g_mux);

        vTaskDelay(10 / portTICK_PERIOD_MS);  // 100 Hz
    }
}

void IMU_StartTask() {
    xTaskCreatePinnedToCore(_imuTask, "IMU_Task", 3072, NULL, 2, NULL, 0);
}

// ── Public getters ────────────────────────────────────────────
float IMU_GetYaw() {
    float v;
    portENTER_CRITICAL(&g_mux);
    v = g_yaw;
    portEXIT_CRITICAL(&g_mux);
    return v;
}

float IMU_GetAccelX() {
    float v;
    portENTER_CRITICAL(&g_mux);
    v = g_accelX;
    portEXIT_CRITICAL(&g_mux);
    return v;
}

float IMU_GetAccelY() {
    float v;
    portENTER_CRITICAL(&g_mux);
    v = g_accelY;
    portEXIT_CRITICAL(&g_mux);
    return v;
}

bool IMU_IsReady() { return g_ready; }
