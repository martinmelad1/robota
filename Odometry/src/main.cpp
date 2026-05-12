#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>


const float WHEEL_RADIUS  = 0.097;   // real one 
const float LX            = ;   // metres — distance from robot centre to left/right wheel
const float LY            = ;   // metres — distance from robot centre to front/back wheel
const int   TICKS_PER_REV = 1496;   // Quadrature (4x) mode got it from  (datasheet)
const int   MAX_PWM       = 200;    // max PWM out of 255 — lower = safer during testing

// =============================================================
//  X positive = right, X negative = left
//  Y positive = forward, Y negative = backward
// =============================================================

// Move 4 — Pure X — RED cube station
const float TARGET_RED_X   = -1.0;
const float TARGET_RED_Y   =  0.0;

// Move 5 — Pure Y — GREEN cube station
const float TARGET_GREEN_X =  0.0;
const float TARGET_GREEN_Y = -1.0;

// Move 6 — Diagonal — BLUE cube station
const float TARGET_BLUE_X  = -0.7;
const float TARGET_BLUE_Y  = -0.7;

// Position tolerance — robot stops when within this distance of target
const float POSITION_TOLERANCE = 0.02;   // 2cm —  for placing cube



// =============================================================
//  GLOBAL VARIABLES
// =============================================================

// Encoder counters — updated by ISR automatically
volatile long ticks_FL = 0;
volatile long ticks_FR = 0;
volatile long ticks_BL = 0;
volatile long ticks_BR = 0;

// Previous tick snapshots for delta calculation
long prev_FL = 0, prev_FR = 0, prev_BL = 0, prev_BR = 0;

// Robot pose — updated by odometry every 10ms
float pose_x     = 0.0;
float pose_y     = 0.0;
float pose_theta = 0.0;   // radians, 0 = direction robot faced when reset

// IMU
MPU6050 mpu;
float gyro_z_offset  = 0.0;
unsigned long last_odo_time = 0;

// PID state
float err_x_prev  = 0.0;
float err_y_prev  = 0.0;
float integral_x  = 0.0;
float integral_y  = 0.0;

// Competition state machine
enum RobotState {
    STATE_IDLE,          // waiting for cube color input
    STATE_MOVE_RED,      // driving to red station   (Pure X)
    STATE_MOVE_GREEN,    // driving to green station  (Pure Y)
    STATE_MOVE_BLUE,     // driving to blue station   (Diagonal)
    STATE_AT_TARGET,     // arrived — arm places cube
    STATE_DONE           // run complete
};

RobotState currentState = STATE_IDLE;
String cubeColor = "";   // "RED", "GREEN", or "BLUE" — set by QR code reader

// =============================================================
//  ISR FUNCTIONS — called automatically on each encoder tick
// =============================================================

void ISR_FL() { ticks_FL += (digitalRead(ENC_FL_B) == HIGH) ? 1 : -1; }
void ISR_FR() { ticks_FR += (digitalRead(ENC_FR_B) == HIGH) ? 1 : -1; }
void ISR_BL() { ticks_BL += (digitalRead(ENC_BL_B) == HIGH) ? 1 : -1; }
void ISR_BR() { ticks_BR += (digitalRead(ENC_BR_B) == HIGH) ? 1 : -1; }

// =============================================================
//  READ GYRO Z
//  Returns rotation speed in radians/second
// =============================================================

float readGyroZ() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    return (gz / 131.0) * (PI / 180.0);   // raw → radians/sec
}

// =============================================================
//  CALIBRATE IMU
//  Keep robot completely still for 1 second during this
// =============================================================

void calibrateIMU() {
    Serial.println("Calibrating IMU — keep robot still for 1 second...");
    float sum = 0.0;
    for (int i = 0; i < 500; i++) {
        sum += readGyroZ();
        delay(2);
    }
    gyro_z_offset = sum / 500.0;
    Serial.print("IMU ready. Offset = ");
    Serial.println(gyro_z_offset, 6);
}

// =============================================================
//  UPDATE ODOMETRY
//  Call every 10ms. Updates pose_x, pose_y, pose_theta.
//    1. dt         = time since last call (seconds)
//    2. d1..d4     = encoder ticks this loop per wheel
//    3. p1..p4     = convert ticks → radians of wheel rotation
//    4. dVx,dVy,dW = forward kinematics → robot-frame movement
//    5. pose_theta = fuse IMU gyro + encoder heading
//    6. dx,dy      = rotate to world frame using current heading
//    7. pose_x,y  += dx,dy
// =============================================================

void updateOdometry() {
    // Step 1: dt
    unsigned long now = millis();
    float dt = (now - last_odo_time) / 1000.0;
    last_odo_time = now;
    if (dt <= 0.0) return;

    // Step 2: delta ticks — safely read volatile counters
    noInterrupts();
    long c1 = ticks_FL, c2 = ticks_FR, c3 = ticks_BL, c4 = ticks_BR;
    interrupts();

    long d1 = c1 - prev_FL;
    long d2 = c2 - prev_FR;
    long d3 = c3 - prev_BL;
    long d4 = c4 - prev_BR;
    prev_FL = c1; prev_FR = c2; prev_BL = c3; prev_BR = c4;

    // Step 3: ticks → radians
    float k  = (2.0 * PI) / TICKS_PER_REV;
    float p1 = d1 * k;
    float p2 = d2 * k;
    float p3 = d3 * k;
    float p4 = d4 * k;

    // Step 4: mecanum forward kinematics
    // dVx = sideways (+ = right), dVy = forward (+ = forward), dW = rotation
    float dVx = (WHEEL_RADIUS / 4.0) * (-p1 + p2 + p3 - p4);
    float dVy = (WHEEL_RADIUS / 4.0) * ( p1 + p2 + p3 + p4);
    float dW  = (WHEEL_RADIUS / (4.0 * (LX + LY))) * (-p1 + p2 - p3 + p4);

    // Step 5: fuse IMU + encoder heading (complementary filter)
    float gyro_z     = readGyroZ() - gyro_z_offset;
    float theta_gyro = pose_theta + gyro_z * dt;
    float theta_enc  = pose_theta + dW;
    pose_theta = 0.98 * theta_gyro + 0.02 * theta_enc;

    // Keep theta in -π to +π
    while (pose_theta >  PI) pose_theta -= 2.0 * PI;
    while (pose_theta < -PI) pose_theta += 2.0 * PI;

    // Step 6: rotate robot-frame → world-frame
    float dx = dVx * cos(pose_theta) - dVy * sin(pose_theta);
    float dy = dVx * sin(pose_theta) + dVy * cos(pose_theta);

    // Step 7: accumulate
    pose_x += dx;
    pose_y += dy;

    // Print to laptop (required by competition rules)
    Serial.print("X:"); Serial.print(pose_x, 3);
    Serial.print(" Y:"); Serial.print(pose_y, 3);
    Serial.print(" TH:"); Serial.print(pose_theta * 180.0 / PI, 1);
    Serial.print("deg");
    Serial.print(" | FL:"); Serial.print(c1);
    Serial.print(" FR:"); Serial.print(c2);
    Serial.print(" BL:"); Serial.print(c3);
    Serial.print(" BR:"); Serial.println(c4);
}

// =============================================================
//  DRIVE MOTORS
//  Vx, Vy, W are -1.0 to +1.0 fractions
//  This is INVERSE kinematics — converts desired motion → wheel speeds
//
//  Mecanum inverse kinematics:
//    FL (front-left)  = -Vx + Vy - W
//    FR(front-right) = +Vx + Vy + W
//    BL (back-left)   = +Vx + Vy - W
//    W4BR (back-right)  = -Vx + Vy + W
// =============================================================

void setMotorPWM(int pin_pwm, int pin_dir, float speed) {
    // speed is -1.0 to +1.0
    // Clamp to safe range
    speed = constrain(speed, -1.0, 1.0);
    int pwm = (int)(abs(speed) * MAX_PWM);
    pwm = constrain(pwm, 0, MAX_PWM);

    digitalWrite(pin_dir, speed >= 0 ? HIGH : LOW);
    analogWrite(pin_pwm, pwm);
}

void driveMotors(float Vx, float Vy, float W) {
    float FL = -Vx + Vy - W;   
    float FR =  Vx + Vy + W;   
    float BL =  Vx + Vy - W;   
    float BR = -Vx + Vy + W;  

    // Normalise if any wheel exceeds 1.0
    float maxVal = max(max(abs(FL), abs(FR)), max(abs(BL), abs(BR)));
    if (maxVal > 1.0) {
        FL /= maxVal; FR /= maxVal; BL /= maxVal; BR/= maxVal;
    }

    setMotorPWM(MOT_FL_PWM, MOT_FL_DIR, FL);
    setMotorPWM(MOT_FR_PWM, MOT_FR_DIR, FR);
    setMotorPWM(MOT_BL_PWM, MOT_BL_DIR, BL);
    setMotorPWM(MOT_BR_PWM, MOT_BR_DIR, BR);
}

void stopMotors() {
    analogWrite(MOT_FL_PWM, 0);
    analogWrite(MOT_FR_PWM, 0);
    analogWrite(MOT_BL_PWM, 0);
    analogWrite(MOT_BR_PWM, 0);
}

// =============================================================
//  MOTION CONTROLLER
//  Call this every loop when in autonomous mode.
//  Drives robot from current pose toward (target_x, target_y).
//  Returns true when robot has arrived within POSITION_TOLERANCE.
//
//  HOW THE PID WORKS:
//    error_x = how far left/right we still need to go
//    error_y = how far forward/back we still need to go
//    PID calculates how hard to push in each axis
//    theta correction keeps robot from rotating during move
// =============================================================

bool driveToTarget(float target_x, float target_y, float dt) {
    // Calculate position errors
    float err_x = target_x - pose_x;
    float err_y = target_y - pose_y;

    // Distance remaining
    float distance = sqrt(err_x * err_x + err_y * err_y);

    // Check if arrived
    if (distance < POSITION_TOLERANCE) {
        stopMotors();
        return true;   // ARRIVED
    }

    // PID for X axis
    integral_x += err_x * dt;
    float deriv_x = (err_x - err_x_prev) / dt;
    float cmd_x = Kp_pos * err_x + Ki_pos * integral_x + Kd_pos * deriv_x;
    err_x_prev = err_x;

    // PID for Y axis
    integral_y += err_y * dt;
    float deriv_y = (err_y - err_y_prev) / dt;
    float cmd_y = Kp_pos * err_y + Ki_pos * integral_y + Kd_pos * deriv_y; //lhd ma nzbt PID ll position
    err_y_prev = err_y;

    // Clamp speed commands
    cmd_x = constrain(cmd_x, -MAX_SPEED, MAX_SPEED);
    cmd_y = constrain(cmd_y, -MAX_SPEED, MAX_SPEED);

    // Apply minimum speed to overcome motor friction
    if (abs(cmd_x) > 0.01 && abs(cmd_x) < MIN_SPEED)
        cmd_x = (cmd_x > 0) ? MIN_SPEED : -MIN_SPEED;
    if (abs(cmd_y) > 0.01 && abs(cmd_y) < MIN_SPEED)
        cmd_y = (cmd_y > 0) ? MIN_SPEED : -MIN_SPEED;

    // Heading correction — fight any rotation using IMU
    // Target heading is 0 (same direction as when autonomous started)
    float theta_err = -pose_theta;   // we want theta = 0 always
    float cmd_w = Kp_theta * theta_err;
    cmd_w = constrain(cmd_w, -0.3, 0.3);

    // Send to motors
    driveMotors(cmd_x, cmd_y, cmd_w);

    return false;   // NOT YET ARRIVED
}

// =============================================================
//  RESET POSE
//  Call when robot reaches (0,0) start position before autonomous.
//  Zeroes out position and heading so targets are relative to here.
// =============================================================

void resetPose() {
    noInterrupts();
    ticks_FL = 0; ticks_FR = 0; ticks_BL = 0; ticks_BR = 0;
    interrupts();
    prev_FL = 0; prev_FR = 0; prev_BL = 0; prev_BR = 0;
    pose_x = 0.0;
    pose_y = 0.0;
    pose_theta = 0.0;
    err_x_prev = 0.0;
    err_y_prev = 0.0;
    integral_x = 0.0;
    integral_y = 0.0;
    Serial.println("Pose reset to (0,0,0). Autonomous ready.");
}

// =============================================================
//  PLACE CUBE — stub function
//  Replace this with your actual arm control code
// =============================================================

void placeCube() {
    Serial.println("Placing cube — run arm sequence here");
    // TODO: add your arm motor commands here
    // Example: lower arm, open gripper, raise arm
    delay(2000);   // placeholder — remove when arm code is added
    Serial.println("Cube placed.");
}

// =============================================================
//  SETUP
// =============================================================

void setup() {
    Serial.begin(115200);
    Serial.println("=== MCT333 Mecanum Competition Robot ===");

    // Motor pins
    pinMode(MOT_FL_PWM, OUTPUT); pinMode(MOT_FL_DIR, OUTPUT);
    pinMode(MOT_FR_PWM, OUTPUT); pinMode(MOT_FR_DIR, OUTPUT);
    pinMode(MOT_BL_PWM, OUTPUT); pinMode(MOT_BL_DIR, OUTPUT);
    pinMode(MOT_BR_PWM, OUTPUT); pinMode(MOT_BR_DIR, OUTPUT);
    stopMotors();

    // Encoder pins
    // If your encoder PCB has its own pull-ups, change to INPUT
    pinMode(ENC_FL_A, INPUT); pinMode(ENC_FL_B, INPUT);
    pinMode(ENC_FR_A, INPUT); pinMode(ENC_FR_B, INPUT);
    pinMode(ENC_BL_A, INPUT); pinMode(ENC_BL_B, INPUT);
    pinMode(ENC_BR_A, INPUT); pinMode(ENC_BR_B, INPUT);

    // Attach interrupts — one per wheel Channel A
    //wecan use change if rising didn't help us reaching the target
    attachInterrupt(digitalPinToInterrupt(ENC_FL_A), ISR_FL, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_FR_A), ISR_FR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_BL_A), ISR_BL, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_BR_A), ISR_BR, RISING);
    Serial.println("Encoders ready.");

    // IMU
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("ERROR: MPU6050 not connected! Check SDA/SCL wiring.");
        while (1);
    }
    Serial.println("MPU6050 connected.");
    delay(500);
    calibrateIMU();

    last_odo_time = millis();
    Serial.println("=== System ready ===");
    Serial.println("Send via Serial:");
    Serial.println("  'R' = reached (0,0), reset pose and start auto");
    Serial.println("  'r' = cube is RED   → move to (-1.0, 0.0)");
    Serial.println("  'g' = cube is GREEN → move to ( 0.0,-1.0)");
    Serial.println("  'b' = cube is BLUE  → move to (-0.7,-0.7)");
    Serial.println("In real competition replace Serial input with QR code reader output.");
}

// =============================================================
//  MAIN LOOP
// =============================================================

void loop() {
    unsigned long now = millis();

    // ── Odometry — runs every 10ms ──────────────────────────────
    static unsigned long last_odo = 0;
    float dt = (now - last_odo) / 1000.0;
    if (now - last_odo >= 10) {
        last_odo = now;
        updateOdometry();
    }

    // ── Serial commands for testing / cube color input ──────────
    
    if (Serial.available()) {
        char cmd = Serial.read();

        if (cmd == 'R') {
            // You have manually driven to (0,0) and left controller
            // Reset pose so autonomous targets are relative to here
            resetPose();
            currentState = STATE_IDLE;
            Serial.println("Send cube color: r=RED  g=GREEN  b=BLUE");
        }
        else if (cmd == 'r') {
            cubeColor = "RED";
            currentState = STATE_MOVE_RED;
            Serial.println("Target: RED station (-1.0, 0.0) — PURE X MOVE");
        }
        else if (cmd == 'g') {
            cubeColor = "GREEN";
            currentState = STATE_MOVE_GREEN;
            Serial.println("Target: GREEN station (0.0, -1.0) — PURE Y MOVE");
        }
        else if (cmd == 'b') {
            cubeColor = "BLUE";
            currentState = STATE_MOVE_BLUE;
            Serial.println("Target: BLUE station (-0.7, -0.7) — DIAGONAL MOVE");
        }
    }

    
}


