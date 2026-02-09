package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.VisionSubsystem;

/**
 * FixedOpMode: Manual control for all robot systems.
 *
 * GAMEPAD 1 & 2 (BOTH):
 * - Left stick Y: Forward/backward
 * - Right stick X: Strafe left/right
 * - Left stick X: Turn left/right
 * - Left trigger: Slow mode
 * - Right trigger: Intake inwards
 * - Right bumper: Intake outwards (outtake)
 *
 * GAMEPAD 2 ONLY (SHOOTER):
 * - Right stick X: Turret rotation (when not driving)
 * - B button: Gate sequence (open → intake 3 sec → close)
 * - Left bumper: Decrease shooter speed (-100 tps)
 *
 * NOTE: Gamepad2 right bumper does BOTH outtake AND shooter speed increase
 *
 * DEFAULTS ON START:
 * - Shooter: 1250 ticks/sec
 * - Gate: 0.45 (closed)
 */
@Config
@TeleOp(name = "Fixed Manual OpMode", group = "Main")
public class FixedOpMode extends LinearOpMode {

    /**
     * FTC Dashboard Configuration
     * Tune these values in real-time at: http://192.168.43.1:8080/dash
     */
    public static class Params {
        // Drive control
        public static double DEADZONE = 0.1;
        public static double SLOW_MODE_FACTOR = 0.4;

        // Intake control
        public static double TRIGGER_THRESHOLD = 0.1;

        // Shooter defaults
        public static double DEFAULT_SHOOTER_VELOCITY = 1400.0;  // Default to close shot
        public static double DEFAULT_GATE_POSITION = 0.45;  // Closed

        // Gate sequence
        public static double GATE_OPEN = 0.25;
        public static double GATE_CLOSED = 0.45;
        public static double GATE_SEQUENCE_DURATION = 3.0;  // seconds

        // Shooter velocity control
        public static double VELOCITY_INCREMENT = 100.0;
        public static double MIN_VELOCITY = 0.0;
        public static double MAX_VELOCITY = 2500.0;
    }

    // Subsystems
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private VisionSubsystem vision;

    // Control state
    private boolean gateSequenceActive = false;
    private ElapsedTime gateTimer;
    private double currentShooterVelocity;

    // Auto-velocity state - remembers last detected distance
    private double lastDetectedDistanceCm = -1.0;  // -1 means no detection yet
    private boolean hasDetectedTag = false;

    // Button edge detection
    private boolean previousB = false;
    private boolean previousRightBumper = false;
    private boolean previousLeftBumper = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Initializing Fixed Manual OpMode...");
        telemetry.addLine("Dashboard available at: http://192.168.43.1:8080/dash");
        telemetry.update();

        // Initialize subsystems
        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, VisionSubsystem.AllianceColor.BLUE);

        // Initialize gate timer
        gateTimer = new ElapsedTime();

        // Set defaults from config
        currentShooterVelocity = Params.DEFAULT_SHOOTER_VELOCITY;

        shooter.setShooterVelocity(Params.DEFAULT_SHOOTER_VELOCITY);
        shooter.setGatePosition(Params.DEFAULT_GATE_POSITION);

        telemetry.addLine("Initialization complete!");
        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ===== VISION UPDATE =====
            vision.update();  // Update AprilTag detection

            // ===== GAMEPAD 1: DRIVE + INTAKE =====
            updateDrive();
            updateIntake();

            // ===== GAMEPAD 2: SHOOTER/TURRET/GATE =====
            updateTurret();
            updateShooterVelocity();
            updateAutoVelocity();  // Auto-adjust velocity based on AprilTag distance
            updateGateSequence();

            // ===== SHOOTER + TURRET UPDATE (CRITICAL - MUST BE CALLED EVERY LOOP) =====
            shooter.update();  // Updates both shooter velocity and turret position hold

            // ===== TELEMETRY =====
            updateTelemetry();
        }

        // Cleanup
        if (vision != null) {
            vision.close();
        }
    }

    /**
     * Drive control (gamepad1 OR gamepad2) - both controllers can drive.
     */
    private void updateDrive() {
        // Combine inputs from both gamepads
        double y = -gamepad1.left_stick_y - gamepad2.left_stick_y;    // Forward/backward
        double x = gamepad1.right_stick_x + gamepad2.right_stick_x;    // Strafe left/right
        double turn = gamepad1.left_stick_x + gamepad2.left_stick_x;   // Turn left/right

        // Apply deadzone
        y = applyDeadzone(y);
        x = applyDeadzone(x);
        turn = applyDeadzone(turn);

        // Slow mode with left trigger (either gamepad)
        if (gamepad1.left_trigger > Params.TRIGGER_THRESHOLD || gamepad2.left_trigger > Params.TRIGGER_THRESHOLD) {
            y *= Params.SLOW_MODE_FACTOR;
            x *= Params.SLOW_MODE_FACTOR;
            turn *= Params.SLOW_MODE_FACTOR;
        }

        drive.driveMecanum(y, x, turn);
    }

    /**
     * Intake control (gamepad1 OR gamepad2).
     * Right trigger = inwards, right bumper = outwards.
     * SKIPS if gate sequence is active (to avoid conflict).
     */
    private void updateIntake() {
        // DON'T control intake if gate sequence is running
        if (gateSequenceActive) {
            return;  // Gate sequence has control
        }

        // Normal intake control - either gamepad can control
        if (gamepad1.right_trigger > Params.TRIGGER_THRESHOLD || gamepad2.right_trigger > Params.TRIGGER_THRESHOLD) {
            // Right trigger (either gamepad) → intake inwards
            intake.intakeIn();
        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            // Right bumper (either gamepad) → intake outwards (outtake)
            intake.intakeOut();
        } else {
            intake.stop();
        }
    }

    /**
     * Turret manual control (gamepad2 right stick X).
     * Holds position with PIDF when joystick is released.
     */
    private void updateTurret() {
        double turretInput = gamepad2.right_stick_x;

        // If joystick is within deadzone, hold position with PIDF
        if (Math.abs(turretInput) < Params.DEADZONE) {
            shooter.holdTurretPosition();
        } else {
            // Manual control when joystick is active
            shooter.manualControl(turretInput);
        }
    }

    /**
     * Shooter velocity control (gamepad2 bumpers).
     */
    private void updateShooterVelocity() {
        boolean rightBumperPressed = detectEdge(gamepad2.right_bumper, previousRightBumper);
        boolean leftBumperPressed = detectEdge(gamepad2.left_bumper, previousLeftBumper);

        previousRightBumper = gamepad2.right_bumper;
        previousLeftBumper = gamepad2.left_bumper;

        if (rightBumperPressed) {
            // Increase shooter velocity
            currentShooterVelocity += Params.VELOCITY_INCREMENT;
            currentShooterVelocity = Math.min(currentShooterVelocity, Params.MAX_VELOCITY);
            shooter.setShooterVelocity(currentShooterVelocity);
        } else if (leftBumperPressed) {
            // Decrease shooter velocity
            currentShooterVelocity -= Params.VELOCITY_INCREMENT;
            currentShooterVelocity = Math.max(currentShooterVelocity, Params.MIN_VELOCITY);
            shooter.setShooterVelocity(currentShooterVelocity);
        }
    }

    /**
     * Auto-adjust shooter velocity based on AprilTag distance.
     * Works with ANY AprilTag (Red or Blue alliance).
     *
     * MEMORY BEHAVIOR:
     * - Remembers last detected distance
     * - Continues using last distance until new detection
     * - Only resets to default (1400) if no tag ever detected
     *
     * VELOCITY RULES:
     * - Distance > 230 cm: 1700 tps (far shot)
     * - Distance ≤ 230 cm: 1400 tps (close shot)
     */
    private void updateAutoVelocity() {
        // Check if ANY AprilTag is visible (not just alliance-specific)
        if (vision.hasAnyTarget()) {
            // Tag is visible - get fresh distance
            double distanceInches = vision.getBestTagRange();

            if (distanceInches >= 0) {
                // Valid distance - convert and remember it
                lastDetectedDistanceCm = distanceInches * 2.54;
                hasDetectedTag = true;
            }
        }

        // Use last known distance (or default if never detected)
        double distanceToUse;
        if (hasDetectedTag && lastDetectedDistanceCm >= 0) {
            // Use last detected distance (even if tag not currently visible)
            distanceToUse = lastDetectedDistanceCm;
        } else {
            // Never detected a tag yet - use close distance (safe default)
            distanceToUse = 200.0;  // Assumes close shot, gives 1400 tps
        }

        // Auto-set velocity based on distance
        if (distanceToUse > 230.0) {
            // Far shot (> 230 cm)
            currentShooterVelocity = 1700.0;
        } else {
            // Close shot (≤ 230 cm)
            currentShooterVelocity = 1400.0;
        }

        shooter.setShooterVelocity(currentShooterVelocity);
    }

    /**
     * Gate sequence (gamepad2 B button).
     * NEW SEQUENCE:
     * 1. Intake starts
     * 2. Gate opens immediately
     * 3. After 3 seconds: intake stops AND gate closes
     */
    private void updateGateSequence() {
        boolean bPressed = detectEdge(gamepad2.b, previousB);
        previousB = gamepad2.b;

        // Start sequence
        if (bPressed && !gateSequenceActive) {
            // STEP 1: Start intake FIRST
            intake.intakeIn();

            // STEP 2: Open gate IMMEDIATELY
            shooter.setGatePosition(Params.GATE_OPEN);

            // STEP 3: Start timer
            gateTimer.reset();
            gateSequenceActive = true;
        }

        // Update sequence - stop everything after 3 seconds
        if (gateSequenceActive) {
            if (gateTimer.seconds() >= Params.GATE_SEQUENCE_DURATION) {
                // Stop intake and close gate at same time
                intake.stop();
                shooter.setGatePosition(Params.GATE_CLOSED);
                gateSequenceActive = false;
            }
        }
    }

    /**
     * Display telemetry.
     */
    private void updateTelemetry() {
        telemetry.addLine("=== FIXED MANUAL OPMODE ===");

        // Drive
        telemetry.addData("Slow Mode", gamepad1.left_trigger > Params.TRIGGER_THRESHOLD ? "ON" : "OFF");

        // Turret
        telemetry.addData("Turret Angle", "%.1f deg", shooter.getCurrentAngle());

        // Shooter/Flywheel (Mentor's one-encoder PIDF approach)
        String velocityMode = "DEFAULT (no tag)";
        if (hasDetectedTag && lastDetectedDistanceCm >= 0) {
            boolean isCurrentlyDetecting = vision.hasAnyTarget();
            String detectionStatus = isCurrentlyDetecting ? "LIVE" : "LAST";

            if (lastDetectedDistanceCm > 230.0) {
                velocityMode = String.format("AUTO-FAR [%s]", detectionStatus);
            } else {
                velocityMode = String.format("AUTO-CLOSE [%s]", detectionStatus);
            }
        }
        telemetry.addData("Velocity Mode", velocityMode);

        // Show last detected distance
        if (hasDetectedTag && lastDetectedDistanceCm >= 0) {
            telemetry.addData("Last Distance", "%.1f cm", lastDetectedDistanceCm);
        }
        telemetry.addData("Flywheel Target", "%.0f tps", currentShooterVelocity);
        telemetry.addData("Flywheel Current", "%.0f tps", shooter.getCurrentShooterVelocity());
        telemetry.addData("Flywheel Power", "%.3f", shooter.getShooterPower());
        telemetry.addData("Flywheel Error", "%.0f tps", shooter.getShooterError());

        // Gate
        telemetry.addData("Gate", gateSequenceActive ? "OPEN (feeding)" : "CLOSED");

        if (gateSequenceActive) {
            telemetry.addData("Gate Timer", "%.1f / %.1f sec",
                    gateTimer.seconds(), Params.GATE_SEQUENCE_DURATION);
        }

        // Vision - AprilTag distance (ANY tag detected)
        telemetry.addLine();
        telemetry.addLine("=== APRILTAG VISION ===");
        if (vision.hasAnyTarget()) {
            // Show closest tag (any alliance)
            int tagId = vision.getBestTagId();
            double distanceInches = vision.getBestTagRange();
            double distanceCm = distanceInches * 2.54;

            telemetry.addData("Tag Status", "DETECTED");
            telemetry.addData("Tag ID", tagId);
            telemetry.addData("Tag Distance", "%.1f cm (%.1f in)", distanceCm, distanceInches);

            double yaw = vision.getYawError();
            if (!Double.isNaN(yaw)) {
                telemetry.addData("Tag Yaw", "%.1f deg", yaw);
            }
        } else {
            telemetry.addData("Tag Status", "NOT VISIBLE");
        }

        // Controls
        telemetry.addLine("--- GAMEPAD 1 & 2 (BOTH) ---");
        telemetry.addLine("Sticks: Drive");
        telemetry.addLine("Left Trigger: Slow mode");
        telemetry.addLine("Right Trigger: Intake IN");
        telemetry.addLine("Right Bumper: Intake OUT");

        telemetry.addLine("--- GAMEPAD 2 (SHOOTER) ---");
        telemetry.addLine("Right Stick X: Turret");
        telemetry.addLine("Left Bumper: Shooter -100");
        telemetry.addLine("B: Gate (3 sec)");

        telemetry.update();
    }

    /**
     * Apply deadzone to joystick input.
     */
    private double applyDeadzone(double value) {
        if (Math.abs(value) < Params.DEADZONE) return 0.0;
        return (value > 0)
                ? (value - Params.DEADZONE) / (1.0 - Params.DEADZONE)
                : (value + Params.DEADZONE) / (1.0 - Params.DEADZONE);
    }

    /**
     * Edge detection for button presses.
     */
    private boolean detectEdge(boolean current, boolean previous) {
        return current && !previous;
    }
}
