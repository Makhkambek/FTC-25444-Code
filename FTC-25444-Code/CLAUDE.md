# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FTC (FIRST Tech Challenge) robotics project for Team 25444 - The Reckless, competing in the Houston regional for the 2024-2025 season. The codebase implements a sophisticated autonomous shooter system with AprilTag vision tracking, turret control, and integrated subsystems.

**Key Technologies:**
- FTC SDK 11.0
- Pedro Pathing 2.0.5 (odometry)
- GoBilda Pinpoint odometry driver
- AprilTag vision processing

## Build Commands

### Building the Project
```bash
# Build debug APK (from root directory)
./gradlew assembleDebug

# Clean build
./gradlew clean assembleDebug

# Install to connected robot controller
./gradlew installDebug
```

### Running on Robot
1. Deploy APK via Android Studio or `gradlew installDebug`
2. On Driver Station, select OpMode:
   - **RED Alliance TeleOp** (AprilTag ID 24)
   - **BLUE Alliance TeleOp** (AprilTag ID 21)
   - **Blue Auto** for autonomous
3. Press INIT, then START

### Testing Individual Subsystems
Use the Testers in `OpModes/Testers/`:
- `[TEST] Turret Tester` - PIDF tuning and Vision tracking
- `[TEST] Shooter Tester` - FSM and motor testing
- `[TEST] Shooter PID Tuner` - Dedicated flywheel PIDF tuning
- `[TEST] Vision Tester` - AprilTag detection
- `[TEST] Servo Tester` - Hardware validation
- `[TEST] Motor Direction Tester` - Verify motor directions

## Architecture Overview

### Three-Layer Design Pattern

**Layer 1: SubSystems** (`SubSystems/`)
- Hardware abstraction and low-level control
- Each subsystem manages its own motors, sensors, servos
- Examples: `Turret`, `Shooter`, `Vision`, `Intake`

**Layer 2: Controllers** (`Controllers/`)
- Business logic and state management
- Coordinate between subsystems and handle gamepad input
- Examples: `TurretController`, `ShooterController`, `HeadingController`, `IntakeController`, `ResetController`

**Layer 3: OpModes** (`OpModes/`)
- Entry points that tie everything together
- Alliance-specific configurations
- Examples: `RedAllianceTeleOp`, `BlueAllianceTeleOp`, `BlueAuto`

### Key Architectural Patterns

**Singleton Pattern for Localizer:**
```java
Localizer localizer = Localizer.getInstance(hardwareMap);
```
- Ensures consistent odometry data across OpMode transitions (Auto → TeleOp)
- Must be initialized FIRST before other subsystems

**Multiple Constructor Pattern in Turret:**
- `Turret(hardwareMap, vision, localizer)` - Full functionality (used in TeleOp/Auto)
- `Turret(hardwareMap, vision)` - Vision-only testing
- `Turret(hardwareMap)` - Basic PIDF tuning without Vision

This allows flexible testing without requiring all dependencies.

## Critical Subsystem Details

### Turret (PIDF-Controlled Rotation)

**Auto-Aim Priority System:**
1. **Vision** (highest priority): Uses `vision.getTargetYaw()` from AprilTag
2. **Odometry** (fallback): Calculates angle using `calculateTargetAngle()` based on robot position and goal coordinates
3. **Manual** (lowest priority): Joystick control updates `targetAngle`

**Key Calibration:**
- `TICKS_PER_DEGREE = 1.2` - Convert encoder ticks to degrees (calibrate in OVERRIDE mode)
- Current PIDF: `kP=0.018, kI=0.0, kD=0.0013, kF=0.002`
- Motor direction: `REVERSE` to make right = positive angles
- Range: ±90° with soft limits

**Important:** The `calculatePIDF()` method includes feedforward (`kF * signum(error)`) to overcome static friction.

### Shooter (FSM-Based Sequence)

**State Machine Flow:**
```
IDLE → SPIN_UP (0.2s) → OPEN_STOP (0.3s) → FEED (1.5s) → RESET → IDLE
```

**Dual Control Systems:**
1. **PIDF for velocity control** - Maintains constant flywheel speed
   - Anti-windup with `INTEGRAL_LIMIT = 100.0`
   - Output smoothing via EMA filter (`SMOOTHING_FACTOR = 0.9`)
   - Rate limiter prevents motor spikes
2. **Dynamic hood adjustment** - Distance-based servo positioning
   - Deadzone of 3cm prevents jitter
   - Interpolates between preset positions based on Vision distance

**Critical:** Always call `updatePID()` in loop even when not shooting - maintains velocity tracking.

### Vision (AprilTag Detection)

**Alliance-Specific Targeting:**
- RED: Tag ID 24
- BLUE: Tag ID 21

**API Design:**
- `getTargetTag()` - Returns detection for current alliance only
- `getBestTag()` - Returns closest tag regardless of alliance (legacy)
- `hasTargetTag()` - Quick check for alliance-specific target

**Integration:** Vision runs continuously and automatically updates both Turret and Shooter hood positioning.

### Robot Class (Master Orchestrator)

**Initialization Order (CRITICAL):**
1. **Localizer** - MUST be first (singleton pattern)
2. **Vision** - Initialize and set alliance before Turret
3. **SubSystems** - DriveTrain, Intake, Shooter, Turret (depends on Vision + Localizer)
4. **Controllers** - Created last, linked to subsystems

**Key Methods:**
- `update(gamepad1, gamepad2, telemetry)` - Main loop called every cycle
  - Updates Localizer
  - Updates DriveTrain (gamepad1)
  - Updates dynamic Hood positioning based on Vision
  - Updates PID for Shooter
  - Delegates controller updates (gamepad2)
- `setAlliance(isRedAlliance)` - Configures alliance-specific behavior

**Controller Coordination:**
- `ResetController` - Options button resets all subsystems to safe state
  - Calls `turret.returnToCenter()`, `shooter.reset()`, `intake.off()`
  - Re-enables auto-aim
  - Resets heading controller

### Localizer (Odometry Singleton)

**GoBilda Pinpoint Driver Configuration:**
```java
pinpoint.setOffsets(-107.95, -63.5, DistanceUnit.MM);
pinpoint.setEncoderResolution(GoBildaOdometryPods.goBILDA_4_BAR_POD);
```

**Critical Methods:**
- `update()` - Call ONCE per loop, updates X, Y, heading
- `updateHeadingOnly()` - Lightweight update for heading-only operations
- `setPosition(x, y, heading)` - Transfer position from Auto to TeleOp

**Heading Normalization:** Maintains continuous heading tracking even across 360° wraps by tracking delta changes.

### DriveTrain (Mecanum Drive with Heading Lock)

**Control Features:**
- **Field-centric controls** - Left stick for movement (X/Y), right stick for rotation
- **Heading lock** - Left bumper toggles heading lock, Right bumper engages PID-controlled heading hold
- **Slow mode** - Right trigger reduces speed to 30% for precision movement

**Heading Lock System:**
- Uses `HeadingController` with PIDF control (`kP=0.024, kI=0.0, kD=0.0003, kF=0.005`)
- When right bumper held, robot maintains current heading using PID
- Left bumper toggles the lock state
- Provides stable rotation control during complex maneuvers

**Integration:** DriveTrain instantiates its own `HeadingController` instance and manages heading lock state internally.

## Hardware Configuration Names

**Motors:**
- `turretMotor` - Turret rotation with encoder
- `shooterMotor1`, `shooterMotor2` - Dual flywheel motors
- `Intake` - Ball collection motor
- `leftFront`, `rightFront`, `leftRear`, `rightRear` - Mecanum drive

**Servos:**
- `shooterHood` - Angle adjustment (0.0-1.0 range)
- `shooterStop` - Ball gate (0.0=closed, 1.0=open)

**Sensors:**
- `Webcam` - AprilTag detection camera
- `pinpoint` - GoBilda Pinpoint odometry sensor

## Emergency Reset System

The `ResetController` provides a one-button emergency recovery mechanism accessible via gamepad2's **Options button**.

**Reset Actions (in order):**
1. Re-enables turret auto-aim
2. Resets heading controller PID state
3. Turns off intake motor
4. Calls `shooter.reset()` - Stops motors, resets servos, returns FSM to IDLE
5. Calls `turret.returnToCenter()` - PID-controlled return to 0° position

**Use Cases:**
- Recovery from jammed mechanisms
- Quick return to known-good state between matches
- Clearing stuck FSM states
- Resetting after manual testing/debugging

**Implementation Note:** The reset is non-blocking - turret returns to center using PID, allowing other operations to continue.

## Common Workflows

### Adding a New Subsystem

1. Create class in `SubSystems/` extending basic pattern:
```java
public class NewSubsystem {
    private DcMotor motor;

    public NewSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "motorName");
        // Initialize hardware
    }

    public void update() {
        // Called every loop
    }
}
```

2. Add controller in `Controllers/` if business logic needed
3. Integrate into `Robot.java` constructor and `update()` method
4. Create tester in `OpModes/Testers/` for validation

### PIDF Tuning Workflow

1. Use TurretTester (or create similar tester)
2. Connect to FTC Dashboard (192.168.43.1:8080)
3. Adjust `KP`, `KI`, `KD`, `KF` in real-time
4. Test with preset positions (Right/Left Bumper for ±45°)
5. Copy final values back to subsystem defaults

**Tuning Order:**
- Start with `kP` until system responds
- Add `kD` to reduce oscillation/overshoot
- Add `kF` if steady-state error exists (doesn't reach target)
- Add `kI` only if absolutely necessary (risk of windup)

### Vision Tracking Calibration

**If turret doesn't track correctly:**

1. Verify correct AprilTag ID for alliance
2. Check `vision.setAlliance(isRedAlliance)` is set correctly
3. In TurretTester, hold Right Trigger and observe telemetry:
   - `Tag Visible: YES` - Vision working
   - `Yaw (turret angle)` - Should match visual angle
   - `Target Angle` - Should equal Yaw when tracking
4. If angles inverted, check `turretMotor.setDirection()`

## Git Workflow

**Active Branches:**
- `main` - Stable competition code
- `dev` - Development and testing
- Feature branches for major changes

**Current State:** Working primarily in `dev` branch with frequent merges to `main`.

## Important Conventions

### Gamepad Assignment
- **Gamepad1**: Driver controls (DriveTrain movement and heading lock)
- **Gamepad2**: Operator controls (Turret, Shooter, Intake)
  - Right Trigger: Manual turret rotation
  - Left/Right Bumpers: Auto-aim toggle and preset angles
  - Options button: Emergency reset all subsystems

### Alliance Color Handling
Always pass `isRedAlliance` boolean rather than magic numbers or strings. This ensures consistency across Vision tag selection and UI displays.

### Non-Blocking Operations
All long-running operations use FSM pattern (see Shooter) to prevent loop blocking. Never use `sleep()` or `Thread.sleep()` in OpMode loop.

### Manual Control with PID
Manual joystick control modifies `targetAngle` and lets PIDF controller track to it, rather than direct power control. This provides smoother, more predictable motion.

### Vision + Odometry Fallback
Systems try Vision first (most accurate), fall back to odometry calculations if Vision unavailable. This dual-system approach improves reliability.

### Coordinate Systems
- Turret angles: 0° = center, positive = right, negative = left
- Field coordinates: Follow FTC standard (defined in Pedro Pathing)
- Heading: Continuous tracking across wraps (not limited to ±180°)

## Testing Strategy

**Incremental Testing Order:**
1. Individual motor directions (MotorDirectionTester)
2. Servo ranges (ServoTester)
3. Vision tag detection (VisionTester)
4. Turret PIDF tuning (TurretTester)
5. Shooter velocity PIDF tuning (ShooterPIDTuner)
6. Shooter FSM (ShooterTester)
7. Full integration (TeleOp)

**Always test in OVERRIDE mode first** when calibrating encoders or checking physical limits to avoid PID fighting against manual input.

## Current Calibration Values

These values are tuned for the current robot configuration:

**Turret:**
- TICKS_PER_DEGREE: 1.2
- kP: 0.018, kI: 0.0, kD: 0.0013, kF: 0.002
- ANGLE_TOLERANCE: 2.0° (for atTarget() checking)

**Shooter:**
- Target velocity: 2200 ticks/sec
- kP: 0.011, kI: 0.0, kD: 0.0, kF: 0.00041
- SPIN_UP_TIME: 0.2s, OPEN_STOP_TIME: 0.3s, FEED_TIME: 1.5s

**HeadingController (DriveTrain):**
- kP: 0.024, kI: 0.0, kD: 0.0003, kF: 0.005
- Controls robot heading lock when right bumper held

**Vision:**
- RED tag: 24, BLUE tag: 21
- Distance conversion: 2.54 (inches → cm)

## File Organization

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── Controllers/          # Business logic layer
│   ├── HeadingController.java     # Robot heading PID lock
│   ├── IntakeController.java      # Intake gamepad mapping
│   ├── ResetController.java       # Emergency reset coordinator
│   ├── ShooterController.java     # Shooter gamepad mapping
│   └── TurretController.java      # Turret auto-aim + manual
├── SubSystems/          # Hardware abstraction layer
│   ├── Robot.java       # Master integration class
│   ├── DriveTrain.java  # Mecanum drive with heading lock
│   ├── Intake.java      # Ball collection
│   ├── Shooter.java     # Flywheel + Hood FSM
│   ├── Turret.java      # PIDF rotation with vision
│   ├── Vision.java      # AprilTag detection
│   └── Localizer.java   # Singleton odometry
├── OpModes/             # Entry points
│   ├── RedAllianceTeleOp.java
│   ├── BlueAllianceTeleOp.java
│   ├── BlueAuto.java
│   ├── MainTeleOp.java  # (Disabled - legacy)
│   └── Testers/         # Individual subsystem tests
│       ├── MotorDirectionTester.java
│       ├── ServoTester.java
│       ├── VisionTester.java
│       ├── TurretTester.java
│       ├── ShooterPIDTuner.java
│       └── ShooterTester.java
└── pedroPathing/        # Path following library
    ├── Constants.java
    └── Tuning.java
```

## Dependencies and External Libraries

**Pedro Pathing Integration:**
- Used for autonomous path following
- Constants configured in `pedroPathing/Constants.java`
- Follower created via `Constants.createFollower(hardwareMap)`

**FTC Dashboard:**
- Access at `192.168.43.1:8080` when connected to robot
- Real-time PIDF tuning via `@Config` annotation
- Telemetry graphing for debugging

## Known Issues and Workarounds

**Pinpoint Odometry Reset:**
When transitioning Auto → TeleOp, use `localizer.setPosition()` to maintain position rather than resetting. The singleton pattern preserves the instance.

**Vision Tag Occlusion:**
If AprilTag becomes occluded mid-match, system automatically falls back to odometry-based angle calculation. No manual intervention required.

**Turret Cable Wrap:**
Physical ±90° limits prevent cable damage. Software enforces these limits in `calculatePIDF()` before applying motor power.
