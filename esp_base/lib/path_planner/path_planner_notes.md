# Path Planner — Notes

## How it works

1. `PathPlanner_Start()` is called when the GUI sends `MODE_AUTO`.
   - Calls `Odometry_ResetPose()` → origin is set to where the robot is right now.
2. Each segment has a fixed drive direction `(Vx, Vy)` and a target distance.
   - The **PID task** handles motor control — path planner just sends direction.
   - Odometry is read every tick to measure accumulated displacement.
   - When displacement ≥ segment distance → stop → advance.
3. Rotate segments watch `|Δθ|` from snapshot instead of distance.
4. At DROP points the planner stops and returns `true` to the StateMachine.
   - StateMachine sends `REACHED:<color>` over UART to the arm.
   - Arm executes its drop sequence (~7.9 s).
   - StateMachine calls `PathPlanner_AcknowledgeAction()` → planner continues.

## Path Table

| Seg | Type   | Direction    | Distance | Drop  |
|-----|--------|--------------|----------|-------|
| 0   | MOVE   | Forward +Y   | 0.60 m   | —     |
| 1   | ROTATE | CCW          | 90°      | —     |
| 2   | MOVE   | Forward +Y   | 1.35 m   | RED   |
| 3   | MOVE   | Backward -Y  | 0.25 m   | —     |
| 4   | MOVE   | Strafe right | 1.70 m   | BLUE  |
| 5   | MOVE   | Strafe left  | 0.20 m   | —     |
| 6   | ROTATE | CW           | 180°     | —     |
| 7   | MOVE   | Forward +Y   | 0.90 m   | —     |
| 8   | MOVE   | Fwd-Left 45° | 1.42 m   | GREEN |

## Tuning Constants (in `path_planner.cpp`)

| Constant    | Default | Description |
|-------------|---------|-------------|
| `MOVE_SPEED`| 0.50    | Vx/Vy fraction passed to PID (0–1) |
| `ROT_SPEED` | 0.35    | Wz fraction passed to PID (0–1) |
| `POS_TOL`   | 0.03 m  | How many metres before target to stop (overshoot buffer) |
| `ROT_TOL`   | 0.06 rad| How many radians before target angle to stop (~3.5°) |

## Coordinate Frame (matches Odometry.cpp)

```
+Y = forward (away from start)
+X = strafe right
+θ = CCW (counter-clockwise)
 θ = 0 at the moment AUTO is pressed
```

## StateMachine Integration

```
StateMachine                     PathPlanner
─────────────────────────────────────────────────────
START_AUTO_DROP_SEQUENCE
  → PathPlanner_Start()

NAVIGATING_TO_DROP (every 20 ms)
  → PathPlanner_Update()
       returns false → still moving
       returns true  → drop point reached

  → ARM_SendReached("red/blue/green")   (UART2 → arm)
  → enter WAIT_FOR_ARM_DROP (10 s timer)

  → PathPlanner_AcknowledgeAction()
       if complete → MANUAL_MODE
       else        → NAVIGATING_TO_DROP
```

## Odometry Source

Path planner reads from `Odometry_GetPose()` — it does not compute
odometry itself. The `OdoTask` (100 Hz, Core 1) continuously integrates
encoder ticks into `(x, y, θ)`. Path planner is a read-only consumer.
