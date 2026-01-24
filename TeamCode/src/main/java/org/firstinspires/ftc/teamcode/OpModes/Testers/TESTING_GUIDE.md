# Testing Guide - FTC Shooter System

## Testing Order (Recommended)

Follow this order to test from basic components to full system integration:

### 1. **Servo Tester** - Basic Hardware Test
**OpMode:** `[TEST] Servo Tester`

**Purpose:** Test individual servos to find correct positions

**What to test:**
- Shooter Hood servo positions (CLOSE=0.0, MIDDLE=0.5, FAR=1.0)
- Shooter Stop servo positions (CLOSE=0.0, OPEN=1.0)

**Controls:**
- Y: Switch between servos
- DPad Up/Down: Fine adjustment (±0.01)
- DPad Left/Right: Coarse adjustment (±0.1)
- A/B/X: Preset positions (0.0, 0.5, 1.0)

**Expected Results:**
- Hood servo moves smoothly to all positions
- Stop servo opens/closes correctly
- Note any positions that need calibration

---

### 2. **Vision Tester** - Camera & AprilTag Detection
**OpMode:** `[TEST] Vision Tester`

**Purpose:** Test AprilTag detection and distance measurement

**What to test:**
- AprilTag detection for RED alliance (Tag ID 11)
- AprilTag detection for BLUE alliance (Tag ID 12)
- Distance accuracy (in cm)
- Yaw angle calculation

**Controls:**
- DPad Up: Switch to RED alliance
- DPad Down: Switch to BLUE alliance

**Expected Results:**
- Camera detects correct AprilTag ID
- Distance readings are accurate
- Yaw angle updates when moving around tag
- Hood suggestions match distance ranges:
  - < 30cm = CLOSE
  - < 50cm = MIDDLE
  - < 100cm = FAR

---

### 3. **Turret Tester** - Motor & PID Control
**OpMode:** `[TEST] Turret Tester`

**Purpose:** Test turret motor and PID control

**What to test:**
- Manual control with PID
- Preset positions (-90°, 0°, +90°)
- Return to center
- Angle limits

**Controls:**

**MANUAL MODE:**
- Right Stick X: Manual control (PID-based)
- DPad Left: Go to -90°
- DPad Up: Go to 0° (Center)
- DPad Right: Go to +90°
- A: Return to center
- B: STOP
- Y: Switch to AUTO mode

**AUTO MODE:** (Vision integration - commented out)
- Currently disabled for basic testing
- Uncomment vision code to enable tracking/scanning

**Expected Results:**
- Turret moves smoothly with PID control
- Angle limits enforced (-90° to +90°)
- Returns to center accurately
- No jittering or oscillation

**Note:** AUTO mode with vision tracking is commented out. Uncomment when vision is confirmed working.

---

### 4. **Shooter Tester** - Motors & FSM
**OpMode:** `[TEST] Shooter Tester`

**Purpose:** Test shooter motors, servos, and shooting sequence

**Two Modes:**

#### **BASIC MODE** - Component Testing
Test individual components:

**Controls:**
- DPad Up: Hood to FAR (1.0)
- DPad Left: Hood to MIDDLE (0.5)
- DPad Down: Hood to CLOSE (0.0)
- A: Toggle shooter motors ON/OFF
- B: RESET ALL
- Y: Switch to ADVANCED mode

**What to test:**
- Hood servo moves to all positions
- Shooter motors spin up correctly
- Stop servo opens/closes (manual test)
- All components reset properly

#### **ADVANCED MODE** - Full FSM Testing
Test complete shooting sequence:

**Controls:**
- A: Start shooting sequence
- B: RESET ALL
- Y: Switch to BASIC mode

**What to test:**
- FSM sequence timing:
  - IDLE → SPIN_UP (0.2s) → OPEN_STOP (0.3s) → FEED (1.5s) → RESET → IDLE
- Motors spin up before feeding
- Stop servo opens at correct time
- Intake activates during FEED state
- Full reset works correctly

**Vision Integration (commented out):**
- Automatic hood adjustment based on distance
- Uncomment when vision is confirmed working

---

## Full System Integration Test

Once all individual tests pass, run:
- **RED Alliance TeleOp** or **BLUE Alliance TeleOp**

**What to verify:**
1. Vision detects correct alliance tag
2. Turret auto-aims or scans
3. Hood auto-adjusts based on distance
4. Shooting sequence works end-to-end
5. Reset button returns everything to start state

---

## Troubleshooting

### Servos not moving:
- Check power connections
- Verify config names match: "shooterHood", "shooterStop"
- Use Servo Tester to find working range

### Vision not detecting tags:
- Check camera connection
- Verify AprilTag IDs (11 for RED, 12 for BLUE)
- Check lighting conditions
- Use Vision Tester to debug

### Turret jittering:
- Tune PID values in Turret.java (kP, kI, kD)
- Check for mechanical binding
- Verify encoder readings are stable

### FSM sequence timing off:
- Adjust time constants in Shooter.java:
  - SPIN_UP_TIME = 0.2s
  - OPEN_STOP_TIME = 0.3s
  - FEED_TIME = 1.5s

---

## Calibration Values

Record these values during testing:

**Servo Positions:**
- Hood CLOSE: _____ (target: 0.0)
- Hood MIDDLE: _____ (target: 0.5)
- Hood FAR: _____ (target: 1.0)
- Stop CLOSE: _____ (target: 0.0)
- Stop OPEN: _____ (target: 1.0)

**Turret:**
- Ticks per degree: _____ (current: 10.0)
- PID kP: _____ (current: 0.03)
- PID kI: _____ (current: 0.0)
- PID kD: _____ (current: 0.01)

**Vision:**
- Distance calibration multiplier: _____ (current: 2.54 for inches to cm)

**Timing:**
- Spin-up time: _____ (current: 0.2s)
- Stop open delay: _____ (current: 0.3s)
- Feed duration: _____ (current: 1.5s)

---

## Testing Checklist

- [ ] Servo Tester - All servos move correctly
- [ ] Vision Tester - Tags detected, distance accurate
- [ ] Turret Tester - Manual control works, PID tuned
- [ ] Turret Tester - AUTO mode (when ready)
- [ ] Shooter Tester - BASIC mode, all components work
- [ ] Shooter Tester - ADVANCED mode, FSM sequence correct
- [ ] Full TeleOp - RED alliance integration
- [ ] Full TeleOp - BLUE alliance integration
- [ ] Reset button works in all states

---

Good luck with testing! 🤖
