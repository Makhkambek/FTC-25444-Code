# FTC Team 25444 - The Reckless
## Houston Competition Robot Code

### About
This repository contains the robot control code for FTC Team **25444 - The Reckless** competing in the Houston regional competition.

**Developer:** Makhkambek Teshabayev
**Season:** 2024-2025
**Framework:** FTC SDK 10.1

---

## System Overview

This codebase implements a complete autonomous shooter system with computer vision tracking and turret control.

### Key Features

- **Vision System**: AprilTag detection and tracking with alliance-specific targeting
- **Turret Control**: PID-based auto-aim with ±90° scanning range
- **Shooter FSM**: Non-blocking finite state machine for shooting sequences
- **Dual Alliance Support**: Separate TeleOp modes for RED and BLUE alliances
- **Comprehensive Testing Suite**: Individual testers for each subsystem

---

## System Architecture

### SubSystems

| Subsystem | Description | Key Features |
|-----------|-------------|--------------|
| **Vision** | AprilTag detection | Alliance-specific tags (11=RED, 12=BLUE), distance measurement |
| **Turret** | Rotational targeting | PID control, auto-tracking, scanning mode, ±90° limits |
| **Shooter** | Ball launching system | FSM-based sequence, auto hood adjustment, dual motors + servos |
| **Intake** | Ball collection | Variable speed control |
| **DriveTrain** | Mecanum drive | Field-oriented control |
| **Localizer** | Position tracking | Odometry-based (Pedro Pathing) |

### Controllers

| Controller | Purpose |
|------------|---------|
| **ShooterController** | Manages shooting sequence and hood auto-adjustment |
| **TurretController** | Switches between manual and auto-aim modes |
| **IntakeController** | Handles intake operations |
| **ResetController** | Emergency reset to safe state |
| **HeadingController** | IMU-based heading control |

---

## Shooting System Details

### FSM Sequence

The shooter operates through a non-blocking finite state machine:

```
IDLE → SPIN_UP (0.2s) → OPEN_STOP (0.3s) → FEED (1.5s) → RESET → IDLE
```

**States:**
1. **IDLE**: Waiting for trigger
2. **SPIN_UP**: Motors spin to full speed (0.2s)
3. **OPEN_STOP**: Stop servo opens (0.3s)
4. **FEED**: Intake feeds ball through (1.5s)
5. **RESET**: Return to safe state

### Hood Auto-Adjustment

Distance-based hood positioning:
- **< 30cm**: CLOSE position (0.0)
- **< 50cm**: MIDDLE position (0.5)
- **< 100cm**: FAR position (1.0)

### Turret Behavior

- **Target Visible**: PID tracking based on yaw error
- **No Target**: Scanning mode (sweeps ±90°)
- **Manual Override**: Joystick control with target-based PID
- **Reset**: Returns to center (0°) position

---

## Project Structure

```
TeamCode/
├── Controllers/
│   ├── HeadingController.java
│   ├── IntakeController.java
│   ├── ShooterController.java
│   ├── TurretController.java
│   └── ResetController.java
│
├── SubSystems/
│   ├── DriveTrain.java
│   ├── Intake.java
│   ├── Localizer.java
│   ├── Robot.java
│   ├── Shooter.java
│   ├── Turret.java
│   └── Vision.java
│
└── OpModes/
    ├── RedAllianceTeleOp.java
    ├── BlueAllianceTeleOp.java
    │
    └── Testers/
        ├── ServoTester.java
        ├── VisionTester.java
        ├── TurretTester.java
        ├── ShooterTester.java
        ├── TESTING_GUIDE.md
        └── README.md
```

---

## Hardware Configuration

### Motors
- `shooterMotor1`: Shooter flywheel motor 1
- `shooterMotor2`: Shooter flywheel motor 2
- `turretMotor`: Turret rotation motor (with encoder)
- `Intake`: Intake motor

### Servos
- `shooterHood`: Hood angle adjustment (0.0 - 1.0)
- `shooterStop`: Ball stop gate (0.0=CLOSE, 1.0=OPEN)

### Sensors
- `Webcam`: AprilTag detection camera
- IMU: Heading control
- Odometry pods: Position tracking (via Localizer)

---

## Usage

### Running TeleOp

1. Select the appropriate alliance on Driver Station:
   - **RED Alliance TeleOp** (targets AprilTag ID 11)
   - **BLUE Alliance TeleOp** (targets AprilTag ID 12)

2. Controls:
   - **Gamepad 1**: Drive control
   - **Gamepad 2**: Shooter/Turret/Intake control

### Gamepad 2 Controls

| Button | Action |
|--------|--------|
| Right Bumper | Start shooting sequence |
| Right Stick X | Manual turret control |
| Options | Emergency reset all systems |

### Auto Features

The following run automatically every loop:
- ✅ Vision detects alliance-specific AprilTag
- ✅ Turret tracks target or scans
- ✅ Hood adjusts based on distance
- ✅ FSM manages shooting sequence

---

## Testing

A comprehensive testing suite is included for hardware validation and calibration.

### Test OpModes

| Tester | Purpose | Location |
|--------|---------|----------|
| **Servo Tester** | Test hood & stop servos | `OpModes/Testers/ServoTester.java` |
| **Vision Tester** | Verify AprilTag detection | `OpModes/Testers/VisionTester.java` |
| **Turret Tester** | Tune PID and test motors | `OpModes/Testers/TurretTester.java` |
| **Shooter Tester** | Test FSM and integration | `OpModes/Testers/ShooterTester.java` |

### Recommended Test Order

1. **Servo Tester** - Verify hardware functionality
2. **Vision Tester** - Confirm AprilTag detection
3. **Turret Tester** - Calibrate PID values
4. **Shooter Tester** (BASIC) - Test individual components
5. **Shooter Tester** (ADVANCED) - Test FSM sequence
6. **Full TeleOp** - Integration test

See `OpModes/Testers/TESTING_GUIDE.md` for detailed testing instructions.

---

## Calibration Values

### Current Settings

**Turret PID:**
- kP: 0.03
- kI: 0.0
- kD: 0.01
- Ticks per degree: 10.0

**Timing:**
- Spin-up: 0.2s
- Stop open: 0.3s
- Feed duration: 1.5s

**Vision:**
- RED Alliance Tag: 11
- BLUE Alliance Tag: 12
- Distance unit conversion: 2.54 (inches to cm)

**Servo Positions:**
- Hood CLOSE: 0.0
- Hood MIDDLE: 0.5
- Hood FAR: 1.0
- Stop CLOSE: 0.0
- Stop OPEN: 1.0

These values can be adjusted in their respective subsystem files.

---

## Key Design Decisions

### Non-Blocking Architecture
All operations use non-blocking FSM patterns to ensure responsive controls and smooth operation.

### Target-Based Manual Control
Manual turret control uses target angles with PID rather than direct power control for smoother, more precise movement.

### Vision Integration
AprilTag detection runs continuously, automatically adjusting hood position and turret aim based on real-time target data.

### Alliance-Specific Operation
Separate TeleOp modes ensure correct AprilTag targeting for each alliance color.

---

## Dependencies

- **FTC SDK**: 10.1
- **Pedro Pathing**: 2.0.5 (for odometry)
- **AprilTag Library**: Built into FTC SDK

---

## Future Enhancements

Potential improvements for future seasons:
- [ ] Velocity-based shooter control
- [ ] Multi-target tracking
- [ ] Autonomous shooting routines
- [ ] Advanced path following integration
- [ ] Machine learning target prediction

---

## Credits

**Team 25444 - The Reckless**

**Lead Programmer:** Makhkambek Teshabayev

**Special Thanks:**
- FTC Community for SDK and documentation
- Pedro Pathing for odometry library
- FIRST Tech Challenge organization

---

## License

This code is released for educational and FTC competition use.

---

## Contact

For questions about this codebase:
- **Team:** The Reckless #25444
- **Competition:** Houston Regional
- **Season:** 2024-2025

---

**Built with passion for FIRST Tech Challenge** 🤖⚙️
