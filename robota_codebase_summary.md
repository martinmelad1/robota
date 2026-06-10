# Robota — Full Codebase Summary

## Project Overview

**Robota** is the embedded firmware for a **mechatronic sorting robot** built around **three ESP32 microcontrollers**, each with a dedicated role:

| Module | Hardware | Role |
|---|---|---|
| `esp_base` | ESP32 (main board) | Brain — motion control, WiFi dashboard, orchestration |
| `esp_arm` | ESP32 (arm board) | Arm servo control, gripper, IK solver |
| `esp_cam` | ESP32-CAM | QR code scanning, ultrasonic distance, JPEG camera server |

All three are **PlatformIO projects** and communicate with each other via **UART** (hardware serial).

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        esp_base (Brain)                         │
│                                                                 │
│  WiFi AP "RobotController"  ←→  Browser Dashboard (WebSocket)  │
│                                                                 │
│  FreeRTOS Tasks (Core 1):                                       │
│    OdoTask    (100 Hz)  — encoder odometry                      │
│    BrainTask  (50 Hz)   — MasterStateMachine.update()           │
│    PIDTask    (100 Hz)  — mecanum wheel drive                   │
│    UART_Cam   (task)    — receives QR_OK / DIST / CAM_IP        │
│    UART_Arm   (task)    — sends JOINT:/GRIP:/REACHED: to arm    │
│  FreeRTOS Tasks (Core 0):                                       │
│    WebBroadcast (10 Hz) — pushes telemetry JSON to dashboard    │
│                                                                 │
│  Queues (inter-task mailboxes):                                 │
│    guiMailbox  — WebSocket/CAM string commands → StateMachine   │
│    pidMailbox  — ChassisMotion struct → PIDTask                 │
│    armMailbox  — ArmMotion struct → UART_Arm_Task               │
└────────────────────┬──────────────────────┬─────────────────────┘
                     │ UART1 (GPIO23/5)      │ UART2 (GPIO0/2)
                     ▼                       ▼
          ┌──────────────────┐    ┌─────────────────────────┐
          │    esp_cam       │    │        esp_arm           │
          │                  │    │                          │
          │  QR scanner      │    │  3-joint servo arm       │
          │  Ultrasonic HC-  │    │  + gripper               │
          │  SR04            │    │  IK solver               │
          │  JPEG camera     │    │  Drop sequence SM        │
          │  server (port80) │    │  Standalone WebServer    │
          └──────────────────┘    └─────────────────────────┘
```

---

## esp_base — Detailed Breakdown

### `src/main.cpp`
The entry point. Responsibilities:
- Creates the WiFi **Access Point** `"RobotController"` (password `robot1234`)
- Serves a dashboard from **LittleFS** (`dashboard.html`, `control.html`) over HTTP
- Exposes a **WebSocket** endpoint at `/ws` for real-time control
- Creates all **FreeRTOS queues** (`guiMailbox`, `pidMailbox`, `armMailbox`)
- Spawns all tasks and starts the PID, UART, and Odometry subsystems
- The `loop()` only cleans WebSocket clients and forwards serial debug commands

---

### `lib/state_machine/`

#### `WorldState.h` — Shared data types
All structs and enums shared across the system:
- `FieldBox` — a box on the arena (QR tag, expected color, position, picked status)
- `StorageSlot` — where the robot stores a picked box (by color, XYZ)
- `DropoffZone` — color-coded drop zone on the arena
- `ChassisMotion` — motion command to PIDTask (type + Vx/Vy/Wz/speed/omega/tickTarget)
- `ArmMotion` — arm joint command (joint_id + direction)
- `StringMessage` — 64-byte string for UART/WebSocket command routing
- `DriveCommand` enum — STOP / FWD / BWD / LEFT / RIGHT / diagonals / ROT / DIRECT / DIST
- `GUITrigger` enum — NONE / TRIGGER_MANUAL / TRIGGER_PICK / TRIGGER_AUTO

#### `StateMachine.h / StateMachine.cpp` — Robot brain
The `MasterStateMachine` class runs at **50 Hz** and implements the full robot logic.

**States (`RobotState` enum):**

| State | Description |
|---|---|
| `MANUAL_MODE` | GUI joystick control, direct chassis/arm commands forwarded to queues |
| `START_PICK_SEQUENCE` | Stops robot, sends `SCAN_QR` to camera, transitions to wait |
| `WAIT_FOR_VISION_QR` | Waits up to 6 s for `QR_OK:<color>` from camera; checks slot not full |
| `WAIT_FOR_ARM_PICK` | Sends `GRIP:CLOSE` to arm to secure the box |
| `WAIT_FOR_ARM_PICK_FINISH` | Waits 2 s, then marks storage slot as `isFull=true` → MANUAL |
| `START_AUTO_DROP_SEQUENCE` | Starts `PathPlanner_Start()`, enters NAVIGATING |
| `NAVIGATING_TO_DROP` | Calls `PathPlanner_Update()` each tick; transitions on drop arrival |
| `WAIT_FOR_ARM_DROP` | Waits 8 s for arm to complete drop, then ACKs path planner |

**Command parsing** in `update()`:
- Reads `StringMessage` from `guiMailbox` each tick
- Parses GUI commands: `MODE_MANUAL`, `MODE_AUTO`, `MODE_PICK`, `FWD`, `BWD`, directional moves, `J1_UP/DOWN`, `J2/J3`, `GRIP_*`, `ARM_STOP`, `SPEED:x.xx`, `PID_TUNE:...`, `QR_OK:<color>`
- In MANUAL_MODE, chassis commands go to `pidMailbox`, arm commands to `armMailbox`

**`getTelemetryJSON()`** — builds a JSON string for the dashboard including:
- Current command, mode, pick status, pick color, camera IP
- Drive speed, odometry pose (x, y, theta)
- Wheel RPM (set vs actual) for all 4 wheels
- PID gains, arm joint angles, ultrasonic distance
- Storage slot status (full/empty) for red/green/blue

---

### `lib/path_planner/`

#### `PathPlanner.h / path_planner.cpp` — Autonomous navigation

Implements a **segment-based open-loop path** for the autonomous drop sequence (RED → BLUE → GREEN).

**Path table (hardcoded, 8 segments):**

| Seg | Type | Direction | Distance | Action |
|---|---|---|---|---|
| 0 | MOVE | Forward | 0.60 m | — |
| 1 | ROTATE | CCW / Left | 90° | — |
| 2 | MOVE | Forward | 1.00 m | **DROP RED** |
| 3 | MOVE | Strafe Right | 1.40 m | **DROP BLUE** |
| 4 | ROTATE | CW / Right | 180° | — |
| 5 | MOVE | Forward | 1.30 m | — |
| 6 | MOVE | Fwd+Right 45° | 1.10 m | **DROP GREEN** |
| 7 | MOVE | Forward | 0.01 m | stop marker |

**How it works:**
- `PathPlanner_Start()` — resets segment index and state
- `PathPlanner_Update()` — called every Brain tick:
  - Snapshots odometry pose at segment start
  - MOVE: drives at `MOVE_SPEED=0.50` until Euclidean distance ≥ `seg.dist - POS_TOL`
  - ROTATE: drives at `ROT_SPEED=0.35` until `|Δθ|` ≥ `seg.dist - ROT_TOL`
  - Non-drop segments auto-advance; drop segments return `true` and wait for ACK
- `PathPlanner_GetPendingAction()` — returns which color to drop
- `PathPlanner_AcknowledgeAction()` — advances to next segment after arm finishes

---

### `lib/PID_Control/`

#### `PID_Control.h / PID_Control.cpp` — Motor driver & velocity loop

> **Note:** Despite the name "PID", this is currently **open-loop** (fixed 255 PWM). The PID infrastructure exists but `Kp/Ki/Kd` are all zero stubs.

**Hardware:**
- 4 DC motors with encoders (FL, FR, RL, RR)
- 4 PWM channels via `ledcSetup/ledcAttachPin`
- Encoder ISRs on `ticksFL/FR/RL/RR` (volatile longs, protected by `tickMux`)
- 2x mode: A+B both on RISING = 748 ticks/rev

**`PID_Compute(Vx, Vy, Wz)`** — mecanum inverse kinematics:
```
tFL = Vy + Vx − Wz
tFR = Vy − Vx + Wz
tRL = Vy − Vx − Wz
tRR = Vy + Vx + Wz
```
Signs determine PWM direction. If `|t| < 0.1` → stop. Otherwise full 255 PWM.

**Hardware workaround:** GPIO 34, 35, 36, 39 can't have pull-ups. If encoder reads 0 while commanded, the system simulates the reading from the commanded value so the dashboard shows correct RPM.

**`PID_TaskCode`** — runs at 100 Hz, reads `pidMailbox`, translates `DriveCommand` enum to Vx/Vy/Wz, supports `CMD_DIST` (distance-based moves via tick count).

---

### `lib/odometry/`

#### `Odometry.h / Odometry.cpp` — Mecanum dead-reckoning

Encoder-only odometry (no IMU currently — MPU6050 placeholder code commented out).

**Parameters:**
- Wheel radius: `48.5 mm` (97 mm diameter)
- Half-track `LX = 0.150 m`, half-wheelbase `LY = 0.150 m`
- Ticks/rev: `748` (2x mode)

**Update cycle (100 Hz):**
1. `dt` from microsecond timer
2. Atomic tick snapshot under `tickMux`
3. Ticks → wheel arc angles (radians)
4. Mecanum FK → robot-frame `dVx`, `dVy`, `dW`
5. Integrate heading: `θ += dW` (wrapped to ±π)
6. Rotate robot-frame displacement to world frame using mid-point heading
7. Accumulate `x`, `y`, `θ` under `poseMux`

**`Odometry_GetPose()`** — thread-safe snapshot used by PathPlanner and telemetry.

---

### `lib/UART_Master/`

#### `UART_Master.h / UART_Master.cpp` — Communication hub

Two dedicated UART ports:
- **UART1** (CAM): TX=GPIO23, RX=GPIO5
- **UART2** (ARM): TX=GPIO0, RX=GPIO2

**Sending to ARM:**
| Function | UART String |
|---|---|
| `ARM_MoveXYZ(x,y,z)` | `MOVE:x.xx,y.yy,z.zz\n` |
| `ARM_Grip(0/1/2)` | `GRIP:OPEN\n` / `GRIP:CLOSE\n` / `GRIP:PICK\n` |
| `ARM_MoveJoint(id, dir)` | `JOINT:id,dir\n` |
| `ARM_SendColor(color)` | `COLOR:color\n` |
| `ARM_SendReached(color)` | `REACHED:color\n` |

**`UART_Cam_Task`** — receives and parses:
- `CAM_IP:x.x.x.x` → stores camera IP for dashboard
- `QR_OK:<color>` → forwards to `guiMailbox` for StateMachine
- `DIST:xx.xx` → stores `ultrasonic_distance_cm`

**`UART_Arm_Task`** — receives `armMailbox`, translates `ArmMotion` → UART strings. Also parses incoming `JOINT_FB:j1,j2,j3` feedback from arm.

---

## esp_arm — Detailed Breakdown

### `src/main.cpp`
- Inits servos, UART slave, and standalone web server
- Spawns `UART_Arm_Task` and `Servo_Control_Task` on Core 1
- `loop()` checks for `new_arm_cmd` flag from UART task and runs `Standalone_Server_Update()`

---

### `lib/UART_Slave_Arm/`

#### `UART_Slave_Arm.cpp` — Command receiver
- UART1 on GPIO17 (TX) / GPIO16 (RX) at 115200 baud
- Reads line-by-line, calls `processArmCommand()`
- **Parsed commands:**
  - `MOVE:x,y,z` — parses floats (logs only, XYZ movement not yet implemented via IK path here)
  - `GRIP:OPEN/CLOSE/PICK` → queues `ServoCommand` to `servoMailbox`
  - `JOINT:id,dir` → queues `ServoCommand`
  - `COLOR:<color>` → acknowledged, no action (pre-announcement)
  - `REACHED:<color>` → calls `Servo_QueueDropSequence(color)` ← **triggers full autonomous drop**
- Sends joint angle feedback `JOINT_FB:j1.0,j2.0,j3.0` every ~100 ms

---

### `lib/servo_control/`

#### `Servo_Control.h / Servo_Control.cpp` — Servo driver + drop state machine

**Hardware pins:**
- J1 (base yaw): GPIO33
- J2 (shoulder): GPIO25
- J3 (elbow): GPIO26
- Gripper: GPIO27

**Joint limits:**
| Joint | Min | Max |
|---|---|---|
| J1 | 0° | 180° |
| J2 | 0° | 50° |
| J3 | 0° | 130° |
| Gripper | 0° | 150° |

**Gripper presets:** OPEN=60°, CLOSE=150°, PICK=90°

**Home position:** J1=90°, J2=50°, J3=130°

**Drop sequence state machine (`DropPhase` enum):**
Triggered by `REACHED:<color>` from base:

| Phase | Action | Duration |
|---|---|---|
| `GOTO_SLOT` | Move arm to IK-solved pick position | 1800 ms |
| `GRIP_CLOSE` | Close gripper on box | 700 ms |
| `GOTO_DROP` | Move arm to IK-solved drop position | 1800 ms |
| `GRIP_OPEN` | Release box | 600 ms |
| `GOTO_FOLD` | Move to compact fold pose | 1200 ms |
| `RETURN_HOME` | Return to home | 1800 ms |
| **Total** | | **≈7900 ms** |

While drop sequence is active, **manual servo commands are blocked**.

**`Servo_Control_Task`** (50 Hz):
1. If drop sequence active → run drop SM, skip manual
2. If drop sequence idle → check `dropSeqMailbox` for new sequence trigger
3. If both idle → process `servoMailbox` for manual dashboard commands
4. Apply continuous sweep (for held buttons) each tick

#### `Arm_IK.h / Arm_IK.cpp` — Inverse Kinematics

3-DOF geometric IK solver (J1 = base yaw, J2 = shoulder, J3 = elbow).

> **⚠ Placeholder status:** `ARM_L1/L2/L3` are all `0.0f` and `theta1/2/3_deg` equations are empty (`0.0f`). The reachability check, clamping, and logging infrastructure is all in place — only the equations need filling in with the actual derived formulas.

**Place functions** (`Place_Red_Box()`, `Place_Blue_Box()`, `Place_Green_Box()`):
- Call `IK_Solve()` for pick, drop, and fold XYZ positions
- If any is UNREACHABLE → abort
- Store results in `gIKCache` (pick/drop/fold poses)
- Push `DropSequenceCmd` to `dropSeqMailbox`

---

### `lib/Standalone_Server/`

#### `Standalone_Server.cpp` — Fallback WiFi control

When the base is **not connected** via UART, the arm hosts its own WiFi AP (`"ArmControl"`, password `robot1234`) with a touch-friendly web UI for manual joint control.

**Connection detection:** monitors GPIO16 (UART RX) via `GPIO_PULLDOWN_ONLY`. If pin was HIGH in the last 1 second → base is connected → **ignore WiFi commands**.

**Embedded HTML/JS UI** (served from PROGMEM):
- Emergency stop button
- Hold-to-move buttons for J1/J2/J3 and gripper sweep
- Tap buttons for GRIP_TAP_OPEN / GRIP_TAP_CLOSE / GRIP_TAP_PICK
- WebSocket reconnect on disconnect

---

## esp_cam — Detailed Breakdown

### `src/main.cpp`
- Connects to base's WiFi AP (`"RobotController"`) as a **station**
- Sends `CAM_IP:x.x.x.x` over UART so the dashboard can show the live feed
- Starts an HTTP server on port 80 serving JPEG snapshots at `/capture`
- `loop()`: checks for `SCAN_QR` UART command, then:
  1. Takes averaged ultrasonic reading → sends `DIST:xx.xx`
  2. Scans for QR code up to 5 seconds → sends `QR_OK:<color>` or `QR_OK:none`

---

### `lib/qr_reader/`

#### `qr_reader.cpp` — QR code scanner

Uses `ESP32QRCodeReader` library on **Core 1** to avoid watchdog issues.

**Init:** disables flash LED (GPIO4 LOW), sets max contrast/brightness, no mirror/flip.

**`QR_Reader_Scan()`** — non-blocking check: calls `receiveQrCode()` with 500 ms timeout, returns decoded string (or empty).

---

### `lib/ultrasonic/`

#### `Ultrasonic.cpp` — HC-SR04 distance sensor

- TRIG=GPIO13, ECHO=GPIO14
- Fires 10µs trigger pulse, measures echo with `pulseIn()` (24 ms timeout)
- `Ultrasonic_Read()` — averages 2 valid samples (with 5 ms gap), returns cm or -1 on timeout

---

### `lib/UART_slave_cam/`

#### `UART_slave_cam.cpp` — UART command receiver for camera

Simple line-based UART reader on UART1. Stores received commands; `UART_CheckForCommand("SCAN_QR")` returns true if that command was seen and clears the flag. `UART_SendResult(str)` writes a response back to the base.

---

## Inter-Module Communication Summary

```
BASE ──SCAN_QR──────────────────────────────► CAM
CAM  ──DIST:xx.xx───────────────────────────► BASE
CAM  ──QR_OK:red/green/blue/none────────────► BASE
CAM  ──CAM_IP:x.x.x.x───────────────────────► BASE

BASE ──JOINT:id,dir─────────────────────────► ARM
BASE ──GRIP:OPEN/CLOSE/PICK─────────────────► ARM
BASE ──REACHED:red/green/blue───────────────► ARM  ← triggers full drop sequence
ARM  ──JOINT_FB:j1,j2,j3────────────────────► BASE ← feedback for dashboard

Browser ──WebSocket─► BASE (guiMailbox) ─► StateMachine ─► pidMailbox / armMailbox
BASE ──WebSocket──────────────────────────────────────────► Browser (telemetry JSON 10Hz)
```

---

## Key Design Notes

1. **Open-loop drive:** Motors run at fixed 255 PWM in the correct direction. PID gains are zeroed — the code supports tuning via `PID_TUNE:vel:FL:Kp:Ki:Kd` commands but no loop correction is active.

2. **Odometry only, no IMU:** The heading is integrated purely from encoder deltas. MPU6050 code is stubbed out.

3. **IK placeholder:** `Arm_IK.cpp` has the full solver skeleton (reachability check, clamping, logging) but the joint angle equations (`theta1/2/3_deg`) are all `0.0f` — they need to be derived from the robot's actual link lengths and geometry.

4. **UART direction swap in StateMachine:** `ROT_L` command maps to `CMD_ROT_R` and vice versa (motor wiring compensation).

5. **Encoder pull-up workaround:** GPIO 34/35/36/39 on ESP32 cannot use internal pull-ups. If no encoder tick is read while a command is active, the code simulates the tick count from the setpoint to keep the dashboard accurate.

6. **Standalone arm fallback:** The arm detects base UART presence by monitoring its own RX pin (GPIO16 pulled down). If UART goes quiet for >1 s, the WiFi AP takes over for standalone testing.
