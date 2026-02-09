package org.firstinspires.ftc.teamcode.Controller;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.VisionSubsystem;

/**
 * ShootingController: Simple manual control for shooter system.
 *
 * GAMEPAD2 CONTROLS:
 * - Right stick X: Manual turret rotation
 * - B button: Gate sequence (open → run intake 3 sec → close)
 * - Right bumper: Increase shooter speed
 * - Left bumper: Decrease shooter speed
 *
 * DEFAULTS ON START:
 * - Shooter: 1250 ticks/sec
 * - Gate: 0.45 (closed)
 */
public class ShootingController {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final VisionSubsystem vision;
    private final Telemetry telemetry;

    private final Gamepad gamepad2;

    // Gate sequence state
    private boolean gateSequenceActive = false;
    private ElapsedTime gateTimer;
    private static final double GATE_SEQUENCE_DURATION = 3.0;  // 3 seconds

    // Default values (set on initialization)
    private static final double DEFAULT_SHOOTER_VELOCITY = 1250.0;
    private static final double DEFAULT_GATE_POSITION = 0.45;  // Closed

    // Gate positions
    private static final double GATE_OPEN = 0.25;
    private static final double GATE_CLOSED = 0.45;

    // Shooter velocity control
    private double currentShooterVelocity = DEFAULT_SHOOTER_VELOCITY;
    private static final double VELOCITY_INCREMENT = 100.0;

    // Button edge detection
    private boolean previousB = false;
    private boolean previousRightBumper = false;
    private boolean previousLeftBumper = false;

    public ShootingController(
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            VisionSubsystem vision,
            Gamepad gamepad2,
            Telemetry telemetry
    ) {
        this.shooter = shooter;
        this.intake = intake;
        this.vision = vision;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        this.gateTimer = new ElapsedTime();

        // Initialize to defaults
        initializeDefaults();
    }

    /**
     * Initialize all systems to default values.
     */
    private void initializeDefaults() {
        shooter.setShooterVelocity(DEFAULT_SHOOTER_VELOCITY);
        shooter.setGatePosition(DEFAULT_GATE_POSITION);

        currentShooterVelocity = DEFAULT_SHOOTER_VELOCITY;
    }

    /**
     * Main update loop - call this every cycle.
     */
    public void update() {
        // Update vision (for future auto-aim use)
        vision.update();

        // ===== TURRET MANUAL CONTROL =====
        double turretInput = gamepad2.right_stick_x;

        // Hold position with PIDF when joystick is released (prevents drift)
        if (Math.abs(turretInput) < 0.1) {
            shooter.holdTurretPosition();  // Lock in place with PIDF
        } else {
            shooter.manualControl(turretInput);  // Manual control
        }

        // ===== SHOOTER VELOCITY CONTROL =====
        handleShooterVelocityControl();

        // ===== GATE SEQUENCE (B BUTTON) =====
        handleGateSequence();

        // ===== SHOOTER + TURRET UPDATE (CRITICAL - MUST BE CALLED EVERY LOOP) =====
        shooter.update();  // Updates both shooter velocity and turret position hold

        // ===== TELEMETRY =====
        updateTelemetry();
    }

    /**
     * Handle shooter velocity adjustment with bumpers.
     */
    private void handleShooterVelocityControl() {
        boolean rightBumperPressed = detectEdge(gamepad2.right_bumper, previousRightBumper);
        boolean leftBumperPressed = detectEdge(gamepad2.left_bumper, previousLeftBumper);

        previousRightBumper = gamepad2.right_bumper;
        previousLeftBumper = gamepad2.left_bumper;

        if (rightBumperPressed) {
            // Increase shooter velocity
            currentShooterVelocity += VELOCITY_INCREMENT;
            currentShooterVelocity = Math.min(currentShooterVelocity, 2500.0);  // Max limit
            shooter.setShooterVelocity(currentShooterVelocity);
        } else if (leftBumperPressed) {
            // Decrease shooter velocity
            currentShooterVelocity -= VELOCITY_INCREMENT;
            currentShooterVelocity = Math.max(currentShooterVelocity, 0.0);  // Min limit
            shooter.setShooterVelocity(currentShooterVelocity);
        }
    }

    /**
     * Handle gate sequence (gamepad2 B button).
     * NEW SEQUENCE:
     * 1. Intake starts
     * 2. Gate opens immediately
     * 3. After 3 seconds: intake stops AND gate closes
     */
    private void handleGateSequence() {
        boolean bPressed = detectEdge(gamepad2.b, previousB);
        previousB = gamepad2.b;

        // Start sequence on B button press
        if (bPressed && !gateSequenceActive) {
            // STEP 1: Start intake FIRST
            intake.intakeIn();

            // STEP 2: Open gate IMMEDIATELY
            shooter.setGatePosition(GATE_OPEN);

            // STEP 3: Start timer
            gateTimer.reset();
            gateSequenceActive = true;

            telemetry.addLine("GATE SEQUENCE STARTED");
        }

        // Update sequence if active - stop everything after 3 seconds
        if (gateSequenceActive) {
            if (gateTimer.seconds() >= GATE_SEQUENCE_DURATION) {
                // Stop intake and close gate at same time
                intake.stop();
                shooter.setGatePosition(GATE_CLOSED);
                gateSequenceActive = false;

                telemetry.addLine("GATE SEQUENCE COMPLETE");
            }
        }
    }

    /**
     * Check if gate sequence is currently active.
     * Use this to avoid intake control conflicts.
     * @return true if gate sequence is running
     */
    public boolean isGateSequenceActive() {
        return gateSequenceActive;
    }

    /**
     * Update telemetry with current status.
     */
    private void updateTelemetry() {
        telemetry.addLine("=== MANUAL SHOOTING CONTROL ===");

        // Turret
        telemetry.addData("Turret Angle", "%.1f deg", shooter.getCurrentAngle());
        telemetry.addData("Turret Ticks", shooter.getTurretCurrentTicks());

        // Shooter/Flywheel motors (Mentor's one-encoder PIDF approach)
        telemetry.addData("Flywheel Target", "%.0f tps", currentShooterVelocity);
        telemetry.addData("Flywheel Current", "%.0f tps", shooter.getCurrentShooterVelocity());
        telemetry.addData("Flywheel Power", "%.3f", shooter.getShooterPower());
        telemetry.addData("Flywheel Error", "%.0f tps", shooter.getShooterError());

        // Gate
        telemetry.addData("Gate", gateSequenceActive ? "OPEN (feeding)" : "CLOSED");

        // Gate sequence
        if (gateSequenceActive) {
            telemetry.addData("Gate Timer", "%.1f / %.1f sec",
                    gateTimer.seconds(), GATE_SEQUENCE_DURATION);
        }

        // Vision - AprilTag info
        telemetry.addLine();
        telemetry.addLine("=== APRILTAG VISION ===");
        telemetry.addData("Alliance Tag Visible", vision.hasStableTarget());
        if (vision.hasStableTarget()) {
            double distance = vision.getTargetDistance();
            if (!Double.isNaN(distance)) {
                telemetry.addData("Tag Distance", "%.1f cm (%.1f in)", distance, distance / 2.54);
            }
            double yaw = vision.getYaw();
            if (!Double.isNaN(yaw)) {
                telemetry.addData("Tag Yaw", "%.1f deg", yaw);
            }
            telemetry.addData("Tag ID", vision.getTargetTagId());
        } else {
            telemetry.addData("Tag Status", "NOT VISIBLE");
        }

        // Controls reminder
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Right Stick X: Turret");
        telemetry.addLine("Bumpers: Shooter speed");
        telemetry.addLine("B: Gate sequence (3 sec)");
    }

    /**
     * Edge detection for button presses.
     */
    private boolean detectEdge(boolean current, boolean previous) {
        return current && !previous;
    }
}
