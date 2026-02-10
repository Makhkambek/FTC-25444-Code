package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ShooterSubsystem {

    /**
     * SHOOTER FLYWHEEL PIDF (MENTOR'S EXACT VALUES)
     * One encoder on Motor1, both motors get same power
     */
    @Config
    public static class Shooter {
        public static double P = 0.0015;
        public static double I = 0.0;
        public static double D = 0.0;
        public static double F = 0.00045;

        public static double SMOOTHING_FACTOR = 0.9;
        public static double MAX_OUTPUT_CHANGE = 0.05;
        public static double INTEGRAL_LIMIT = 100.0;
        public static double MAX_DERIVATIVE = 500.0;
    }

    /**
     * TURRET PIDF (MENTOR'S EXACT VALUES)
     * For vision auto-aim and manual control
     */
    @Config
    public static class Turret {
        public static double P = 0.035;
        public static double I = 0.0;
        public static double D = 0.008;
        public static double F = 0.0015;

        public static double SMOOTHING_FACTOR = 0.15;
        public static double POWER_SMOOTHING = 0.3;
        public static double INTEGRAL_LIMIT = 50.0;
    }

    /**
     * TURRET POSITION HOLD PID
     * Used when holding position (RUN_TO_POSITION mode)
     * Much stronger than auto-aim PID to resist external forces
     */
    @Config
    public static class TurretHold {
        public static double P = 0.010;   // Very strong - resists position error
        public static double I = 0;    // Small integral to eliminate steady-state error
        public static double D = 0;    // Damping to prevent oscillation
        public static double F = 0.07;    // Not needed for position control
    }

    /**
     * SERVO POSITIONS
     */
    @Config
    public static class Servos {
        public static double GATE_CLOSED = 0.22;
        public static double GATE_OPEN = 0.45;
    }

    /**
     * OTHER CONSTANTS
     */
    public static double TICKS_PER_DEGREE = 2.0;
    public static int TURRET_MIN_TICKS = (int)(-90 * TICKS_PER_DEGREE);
    public static int TURRET_MAX_TICKS = (int)(90 * TICKS_PER_DEGREE);
    public static double TURRET_MIN_ANGLE_DEG = -90.0;
    public static double TURRET_MAX_ANGLE_DEG = 90.0;
    public static double DEFAULT_VELOCITY = 1500.0;
    public static double MIN_TARGETING_VELOCITY = 1200.0;

    private final DcMotorEx turretMotor;
    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
    private final Servo gateServo;

    // Turret PIDF control state
    private double targetAngle = 0.0;           // Target angle in degrees
    private double smoothedTargetAngle = 0.0;   // Smoothed target (EMA filtered)
    private double integral = 0.0;              // Integral accumulator
    private double lastError = 0.0;             // Last error for derivative
    private double lastPower = 0.0;             // Last power for smoothing

    // Turret position hold state
    private boolean holdPositionActive = false;
    private int holdTargetTicks = 0;

    // SHOOTER PIDF control state (MENTOR'S APPROACH: One encoder, manual PID)
    private double shooterTargetVelocity = 0.0;     // Target velocity in ticks/sec
    private double shooterLastError = 0.0;          // Last error for derivative
    private double shooterIntegralSum = 0.0;        // Integral accumulator
    private double shooterSmoothedOutput = 0.0;     // Smoothed power output
    private long shooterLastUpdateTime = 0;         // For delta time calculation

    public ShooterSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        gateServo = hardwareMap.get(Servo.class, "gate");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Mentor's approach: manual PIDF control
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ===== MENTOR'S SHOOTER SETUP: ONE ENCODER, MANUAL PIDF =====
        // Motor1 has encoder for velocity feedback
        // Motor2 receives same power as Motor1 (synchronized via opposite directions)
        // Manual PIDF loop calculates power based on Motor1 velocity

        // Shooter Motor 1 - Has encoder, used for velocity feedback
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Manual control
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Shooter Motor 2 - No encoder feedback, mirrors Motor1 power
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);  // Opposite direction
        shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Manual control
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize shooter PID state
        shooterLastUpdateTime = System.nanoTime();

        setGateClosed();
    }

    public void setTurretTargetTicks(int ticks) {
        int clampedTicks = Math.max(TURRET_MIN_TICKS, Math.min(TURRET_MAX_TICKS, ticks));
        turretMotor.setTargetPosition(clampedTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1.0);  // Maximum power for auto-aim
    }

    public int getTurretCurrentTicks() {
        return turretMotor.getCurrentPosition();
    }

    /**
     * Enable position hold mode - turret will resist movement.
     * Uses strong manual PID control for stiff position holding.
     * Call update() or updateTurretPositionHold() every loop to maintain position.
     */
    public void holdTurretPosition() {
        if (!holdPositionActive) {
            // Entering hold mode - save current position as target
            holdTargetTicks = getTurretCurrentTicks();
            holdPositionActive = true;

            // Switch to RUN_USING_ENCODER for manual PID control
            if (turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    /**
     * Disable position hold mode.
     */
    public void releasePositionHold() {
        holdPositionActive = false;
    }

    public void stopTurret() {
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setPower(0.0);
    }

    public void setTurretManualPower(double power) {
        if (turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        int currentTicks = getTurretCurrentTicks();

        if (currentTicks >= TURRET_MAX_TICKS && power > 0) {
            turretMotor.setPower(0);
        } else if (currentTicks <= TURRET_MIN_TICKS && power < 0) {
            turretMotor.setPower(0);
        } else {
            turretMotor.setPower(power);
        }
    }

    // ===== MENTOR'S SHOOTER CONTROL: ONE ENCODER, MANUAL PIDF =====

    /**
     * Set target velocity for shooter motors.
     * Resets PID state when target changes.
     *
     * IMPORTANT: Call updateShooterPID() periodically in your OpMode loop!
     *
     * @param ticksPerSecond Target velocity in encoder ticks per second
     */
    public void setShooterVelocity(double ticksPerSecond) {
        if (Math.abs(shooterTargetVelocity - ticksPerSecond) > 1.0) {
            // Target changed significantly, reset PID state
            shooterTargetVelocity = ticksPerSecond;
            shooterLastError = 0.0;
            shooterIntegralSum = 0.0;
            shooterSmoothedOutput = 0.0;
            shooterLastUpdateTime = System.nanoTime();
        } else {
            shooterTargetVelocity = ticksPerSecond;
        }
    }

    /**
     * Update shooter PIDF control loop (MENTOR'S EXACT ALGORITHM).
     *
     * *** MUST BE CALLED EVERY LOOP IN YOUR OPMODE! ***
     *
     * This implements manual velocity control:
     * 1. Read current velocity from Motor1 encoder
     * 2. Calculate PIDF output
     * 3. Apply smoothing and rate limiting
     * 4. Set same power to both motors
     */
    public void updateShooterPID() {
        // Get current velocity from Motor1 encoder only
        // Use absolute value since Motor1 is REVERSE direction
        double currentVelocity = Math.abs(shooterMotor1.getVelocity());

        // Calculate delta time
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - shooterLastUpdateTime) / 1_000_000_000.0;  // Convert to seconds
        shooterLastUpdateTime = currentTime;

        // Prevent divide-by-zero on first call
        if (deltaTime < 0.001) {
            deltaTime = 0.02;  // Assume 50Hz if too small
        }

        // Calculate error
        double error = shooterTargetVelocity - currentVelocity;

        // Derivative with clamping
        double derivative = (error - shooterLastError) / deltaTime;
        derivative = Math.max(-Shooter.MAX_DERIVATIVE, Math.min(Shooter.MAX_DERIVATIVE, derivative));

        // Integral with anti-windup
        shooterIntegralSum += error * deltaTime;
        shooterIntegralSum = Math.max(-Shooter.INTEGRAL_LIMIT, Math.min(Shooter.INTEGRAL_LIMIT, shooterIntegralSum));

        // PIDF calculation
        double output = (Shooter.P * error) +
                       (Shooter.I * shooterIntegralSum) +
                       (Shooter.D * derivative) +
                       (Shooter.F * shooterTargetVelocity);

        // Apply EMA smoothing (0.9 = heavily weight previous output)
        double newSmoothedOutput = (Shooter.SMOOTHING_FACTOR * shooterSmoothedOutput) +
                                  ((1.0 - Shooter.SMOOTHING_FACTOR) * output);

        // Rate limiter - prevent sudden changes
        double outputChange = newSmoothedOutput - shooterSmoothedOutput;
        if (Math.abs(outputChange) > Shooter.MAX_OUTPUT_CHANGE) {
            newSmoothedOutput = shooterSmoothedOutput + Math.signum(outputChange) * Shooter.MAX_OUTPUT_CHANGE;
        }

        shooterSmoothedOutput = newSmoothedOutput;

        // Clamp output to [0, 1]
        shooterSmoothedOutput = Math.max(0.0, Math.min(1.0, shooterSmoothedOutput));

        // Apply SAME power to BOTH motors
        shooterMotor1.setPower(shooterSmoothedOutput);
        shooterMotor2.setPower(shooterSmoothedOutput);

        // Update state
        shooterLastError = error;
    }

    /**
     * Set shooter to default velocity.
     */
    public void setShooterDefaultVelocity() {
        setShooterVelocity(DEFAULT_VELOCITY);
    }

    /**
     * Stop both shooter motors.
     */
    public void stopShooter() {
        setShooterVelocity(0.0);
    }

    /**
     * Get current shooter velocity (from Motor1 encoder).
     * Returns POSITIVE value even though motor is REVERSE direction.
     * @return Velocity in ticks per second (always positive)
     */
    public double getCurrentShooterVelocity() {
        // Motor1 is REVERSE direction, so negate to get positive velocity
        return Math.abs(shooterMotor1.getVelocity());
    }

    /**
     * Get target shooter velocity.
     * @return Target velocity in ticks per second
     */
    public double getTargetShooterVelocity() {
        return shooterTargetVelocity;
    }

    /**
     * Get current shooter power output (smoothed).
     * @return Power value [0.0, 1.0]
     */
    public double getShooterPower() {
        return shooterSmoothedOutput;
    }

    /**
     * Get shooter velocity error.
     * @return Error in ticks per second
     */
    public double getShooterError() {
        return shooterLastError;
    }

    /**
     * Check if shooter is at target velocity (within tolerance).
     * @param tolerance Acceptable error in ticks/sec
     * @return true if within tolerance
     */
    public boolean isShooterAtSpeed(double tolerance) {
        return Math.abs(shooterLastError) < tolerance;
    }

    /**
     * Set gate servo to a specific position.
     * @param position Servo position (0.0 to 1.0)
     */
    public void setGatePosition(double position) {
        gateServo.setPosition(position);
    }

    /**
     * Open the gate to allow game elements through.
     */
    public void setGateOpen() {
        gateServo.setPosition(Servos.GATE_OPEN);
    }

    /**
     * Close the gate to prevent game elements from passing.
     */
    public void setGateClosed() {
        gateServo.setPosition(Servos.GATE_CLOSED);
    }

    // ===== TURRET CONTROL - MENTOR'S APPROACH =====

    /**
     * Update turret position hold (if active).
     * Uses strong manual PID to resist movement.
     *
     * *** MUST BE CALLED EVERY LOOP WHEN USING holdTurretPosition()! ***
     */
    public void updateTurretPositionHold() {
        if (!holdPositionActive) {
            return;  // Not in hold mode, nothing to do
        }

        // Calculate position error
        int currentTicks = getTurretCurrentTicks();
        double error = holdTargetTicks - currentTicks;

        // Simple PD control with strong gains
        double power = (TurretHold.P * error) + (TurretHold.D * (error - lastError));

        // Clamp power to maximum
        power = Math.max(-1.0, Math.min(1.0, power));

        // Apply power
        turretMotor.setPower(power);

        // Update state
        lastError = error;
    }

    /**
     * Combined update method - call this every loop in your OpMode.
     * Updates both shooter velocity control and turret position hold.
     */
    public void update() {
        updateShooterPID();
        updateTurretPositionHold();
    }

    /**
     * Manual turret control from joystick.
     * Uses direct power control (bypasses PIDF).
     *
     * @param joystickX Joystick input (-1.0 to 1.0)
     */
    public void manualControl(double joystickX) {
        // Disable position hold when manually controlling
        holdPositionActive = false;

        // Apply deadzone
        final double JOYSTICK_DEADZONE = 0.1;
        if (Math.abs(joystickX) < JOYSTICK_DEADZONE) {
            joystickX = 0.0;
        }

        // Scale power (50% max for manual)
        double power = -joystickX * 0.5;

        // Check limits
        int currentTicks = getTurretCurrentTicks();
        double currentAngle = getCurrentAngle();

        if (currentAngle >= TURRET_MAX_ANGLE_DEG && power > 0) {
            power = 0;
        } else if (currentAngle <= TURRET_MIN_ANGLE_DEG && power < 0) {
            power = 0;
        }

        turretMotor.setPower(power);

        // Reset PIDF state when in manual
        lastError = 0.0;
        integral = 0.0;
        lastPower = 0.0;
    }

    /**
     * Auto-aim using vision (MENTOR'S EXACT ALGORITHM).
     *
     * Algorithm:
     * 1. Get yaw error from vision (horizontal offset in degrees)
     * 2. Add yaw to current angle to get new target
     * 3. Apply EMA smoothing
     * 4. Use PIDF control to reach target smoothly
     *
     * @param yaw Yaw error from vision (degrees, positive = tag to RIGHT)
     * @param hasTargetTag True if alliance-specific AprilTag is visible
     */
    public void autoAim(double yaw, boolean hasTargetTag) {
        if (hasTargetTag && !Double.isNaN(yaw)) {
            // STEP 1: Get current turret angle
            double currentAngle = getCurrentAngle();

            // STEP 2: Calculate new target angle
            // yaw is ERROR (offset from center), NOT absolute angle
            // Turret must compensate this error by ADDING it to current angle
            double newTarget = currentAngle + yaw;

            // STEP 3: Clamp to physical limits
            newTarget = Math.max(TURRET_MIN_ANGLE_DEG, Math.min(TURRET_MAX_ANGLE_DEG, newTarget));

            // STEP 4: Apply EMA smoothing (prevents jitter from AprilTag detection noise)
            smoothedTargetAngle += Turret.SMOOTHING_FACTOR * (newTarget - smoothedTargetAngle);
            smoothedTargetAngle = Math.max(TURRET_MIN_ANGLE_DEG, Math.min(TURRET_MAX_ANGLE_DEG, smoothedTargetAngle));

            // STEP 5: Set as target
            targetAngle = smoothedTargetAngle;
        }
        // If no target, keep last target angle (turret holds position)

        // STEP 6: Apply PIDF control to reach target
        double currentAngle = getCurrentAngle();
        double power = calculatePIDF(targetAngle, currentAngle);
        turretMotor.setPower(power);
    }

    /**
     * PIDF control calculation (MENTOR'S EXACT IMPLEMENTATION).
     *
     * @param target Target angle in degrees
     * @param current Current angle in degrees
     * @return Motor power (-0.8 to 0.8)
     */
    private double calculatePIDF(double target, double current) {
        double error = target - current;

        // Anti-windup: limit integral accumulation
        integral += error;
        integral = Math.max(-Turret.INTEGRAL_LIMIT, Math.min(Turret.INTEGRAL_LIMIT, integral));

        // Reset integral when crossing zero (helps remove overshoot)
        if (Math.signum(error) != Math.signum(lastError) && lastError != 0) {
            integral = 0;
        }

        double derivative = error - lastError;
        lastError = error;

        // PIDF = PID + Feedforward
        // kF helps overcome static friction
        double feedforward = (Math.abs(error) > 1.0) ? Turret.F * Math.signum(error) : 0;
        double rawPower = (Turret.P * error) + (Turret.I * integral) + (Turret.D * derivative) + feedforward;

        // Smooth power output (reduces jerkiness)
        double smoothedPower = lastPower + Turret.POWER_SMOOTHING * (rawPower - lastPower);
        lastPower = smoothedPower;

        // Deadband - don't twitch motor when very close
        if (Math.abs(error) < 1.0 && Math.abs(smoothedPower) < 0.05) {
            return 0.0;
        }

        // Limit max power
        return Math.max(-0.8, Math.min(0.8, smoothedPower));
    }

    /**
     * Get current turret angle in degrees.
     *
     * @return Current angle (0 = center, positive = right, negative = left)
     */
    public double getCurrentAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    /**
     * Get current turret power (for telemetry).
     *
     * @return Last applied power
     */
    public double getCurrentTurretPower() {
        return lastPower;
    }

    /**
     * Get last PIDF error (for telemetry).
     *
     * @return Error in degrees
     */
    public double getLastError() {
        return lastError;
    }

    /**
     * Get target angle (for telemetry).
     *
     * @return Target angle in degrees
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Get smoothed target angle (for telemetry).
     *
     * @return Smoothed target angle in degrees
     */
    public double getSmoothedTargetAngle() {
        return smoothedTargetAngle;
    }
}