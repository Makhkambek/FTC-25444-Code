# FTC Shooter System - Test OpModes

Quick reference for testing robot subsystems.

## Available Testers

### 1. **[TEST] Servo Tester**
- Test individual servos (hood, stop)
- Find correct positions
- Fine and coarse adjustment

### 2. **[TEST] Vision Tester**
- AprilTag detection
- Distance measurement
- Alliance switching (RED/BLUE)

### 3. **[TEST] Turret Tester**
- Manual PID control
- Preset positions
- Auto-aim (commented out for basic testing)

### 4. **[TEST] Shooter Tester**
- **BASIC Mode:** Individual components
- **ADVANCED Mode:** Full FSM sequence
- Vision integration (commented out)

## Testing Order

1. Servo Tester → Find correct positions
2. Vision Tester → Verify detection
3. Turret Tester → Tune PID
4. Shooter Tester → Test sequence
5. Full TeleOp → Integration test

## Full Documentation

See **TESTING_GUIDE.md** for detailed instructions, controls, and troubleshooting.

## Quick Start

1. Run **Servo Tester** first to verify hardware
2. Follow the testing order above
3. Record calibration values
4. Run full TeleOp when all tests pass

---

**Note:** Vision and auto-aim features are commented out in some testers for basic hardware testing. Uncomment when ready for integration testing.
