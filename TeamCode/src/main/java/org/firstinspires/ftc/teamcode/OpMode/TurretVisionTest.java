package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.VisionSubsystem;

@TeleOp(name = "Turret Vision Test", group = "Test")
public class TurretVisionTest extends LinearOpMode {

    private enum TurretState {
        MANUAL,
        TARGET_MODE_SEARCHING,  // Target mode ON, but manual control until tag detected
        AUTO_AIM,               // Tag detected, turret auto-rotating
        LOCKED                  // Locked on target
    }

    private static final double YAW_LOCK_THRESHOLD = 5.0;  // Lock when yaw < 5 degrees
    private static final double TICKS_PER_DEGREE = 2.0;
    private static final double JOYSTICK_DEADZONE = 0.1;

    // Two-stage control for stability
    private static final double LARGE_ERROR_THRESHOLD = 15.0;  // degrees
    private static final double LARGE_ERROR_POWER = 0.25;      // Slow for large errors
    private static final double SMALL_ERROR_MAX_POWER = 0.35;  // Max for proportional control
    private static final double PROPORTIONAL_GAIN = 0.02;      // P gain

    private ShooterSubsystem shooter;
    private VisionSubsystem vision;
    private TurretState currentState;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        telemetry.addLine("Initializing...");
        telemetry.update();

        shooter = new ShooterSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, VisionSubsystem.AllianceColor.RED);

        currentState = TurretState.MANUAL;
        boolean previousRightTrigger = false;
        boolean previousDpadUp = false;
        boolean previousDpadDown = false;

        telemetry.addLine("Ready! Press START");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Right Stick X: Manual turret control");
        telemetry.addLine("  Right Trigger: Toggle target mode ON/OFF");
        telemetry.addLine("  Dpad Up: RED alliance (Tag 24)");
        telemetry.addLine("  Dpad Down: BLUE alliance (Tag 20)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update vision
            vision.update();

            // Switch alliance
            boolean dpadUpPressed = gamepad1.dpad_up && !previousDpadUp;
            boolean dpadDownPressed = gamepad1.dpad_down && !previousDpadDown;
            previousDpadUp = gamepad1.dpad_up;
            previousDpadDown = gamepad1.dpad_down;

            if (dpadUpPressed) {
                vision.setAlliance(VisionSubsystem.AllianceColor.RED);
                telemetry.addLine("Switched to RED alliance (Tag 24)");
            }
            if (dpadDownPressed) {
                vision.setAlliance(VisionSubsystem.AllianceColor.BLUE);
                telemetry.addLine("Switched to BLUE alliance (Tag 20)");
            }

            // Toggle target mode with right trigger
            boolean rightTriggerPressed = (gamepad1.right_trigger > 0.1) && !previousRightTrigger;
            previousRightTrigger = (gamepad1.right_trigger > 0.1);

            if (rightTriggerPressed) {
                if (currentState == TurretState.MANUAL) {
                    // Enter target mode - keep manual control until tag detected
                    currentState = TurretState.TARGET_MODE_SEARCHING;
                    telemetry.addLine(">>> TARGET MODE ON - Manual control until tag detected <<<");
                } else {
                    // Exit target mode - return to manual
                    currentState = TurretState.MANUAL;
                    shooter.setTurretManualPower(0.0);
                    telemetry.addLine(">>> TARGET MODE OFF - Manual mode <<<");
                }
            }

            // State machine
            switch (currentState) {
                case MANUAL:
                    handleManual();
                    break;

                case TARGET_MODE_SEARCHING:
                    handleTargetModeSearching();
                    break;

                case AUTO_AIM:
                    handleAutoAim();
                    break;

                case LOCKED:
                    handleLocked();
                    break;
            }

            // Display telemetry
            updateTelemetry();
        }

        // Cleanup
        vision.close();
    }

    private void handleManual() {
        // Manual turret control
        double turretPower = -gamepad1.right_stick_x;
        if (Math.abs(turretPower) > JOYSTICK_DEADZONE) {
            shooter.setTurretManualPower(turretPower * 0.5);
        } else {
            shooter.setTurretManualPower(0.0);
        }
    }

    private void handleTargetModeSearching() {
        // Target mode is ON, but driver still has manual control
        // Waiting for camera to detect the alliance tag
        telemetry.addLine(">>> TARGET MODE - SEARCHING <<<");
        telemetry.addData("Looking for Tag ID", vision.getTargetTagId());

        // Check if alliance tag detected
        if (vision.hasTargetTag()) {
            // Tag detected! Switch to auto-aim
            currentState = TurretState.AUTO_AIM;
            telemetry.addLine("TAG DETECTED - Going AUTO!");
            return;
        }

        // No tag yet - show what we see
        if (vision.hasAnyTarget()) {
            telemetry.addData("Wrong tag detected", vision.getBestTagId());
            telemetry.addLine("Keep searching for correct tag!");
        } else {
            telemetry.addLine("No tags detected - use stick to aim");
        }

        // Allow manual control to help aim toward the tag
        double turretPower = -gamepad1.right_stick_x;
        if (Math.abs(turretPower) > JOYSTICK_DEADZONE) {
            shooter.setTurretManualPower(turretPower * 0.5);
        } else {
            shooter.setTurretManualPower(0.0);
        }
    }

    private void handleAutoAim() {
        // Check if alliance tag is still visible
        if (!vision.hasTargetTag()) {
            // Tag lost - go back to searching with manual control
            shooter.setTurretManualPower(0.0);
            currentState = TurretState.TARGET_MODE_SEARCHING;
            telemetry.addLine("TAG LOST - Back to manual searching");
            return;
        }

        // Tag detected - auto aim using DIRECT POWER CONTROL
        telemetry.addLine(">>> AUTO AIMING - TURRET MOVING <<<");
        double yawDegrees = vision.getTargetYaw();

        if (Double.isNaN(yawDegrees)) {
            yawDegrees = 0.0;
            telemetry.addLine("WARNING: Yaw is NaN!");
        }

        telemetry.addData("Yaw Error", "%.2f deg", yawDegrees);
        telemetry.addData("Current Position", shooter.getTurretCurrentTicks());

        // Two-stage control for stability
        // CRITICAL: Negate yaw because positive yaw = tag to LEFT, need negative power to rotate left
        double turretPower;

        if (Math.abs(yawDegrees) < YAW_LOCK_THRESHOLD) {
            // Locked on - stop moving
            turretPower = 0.0;
            currentState = TurretState.LOCKED;
            gamepad1.rumble(500);
            telemetry.addLine("*** LOCKED ON TARGET ***");
        } else if (Math.abs(yawDegrees) > LARGE_ERROR_THRESHOLD) {
            // Large error: use slow constant power to avoid losing the tag
            // If yaw > 0 (tag to left), apply negative power (rotate left)
            turretPower = (yawDegrees > 0) ? -LARGE_ERROR_POWER : LARGE_ERROR_POWER;
            telemetry.addLine("Large error - slow approach");
        } else {
            // Small error: proportional control for precision
            // Negate yaw so turret rotates TOWARD tag, not away from it
            turretPower = -yawDegrees * PROPORTIONAL_GAIN;

            // Clamp to small error max
            if (turretPower > SMALL_ERROR_MAX_POWER) turretPower = SMALL_ERROR_MAX_POWER;
            if (turretPower < -SMALL_ERROR_MAX_POWER) turretPower = -SMALL_ERROR_MAX_POWER;

            // Minimum power to overcome friction
            if (Math.abs(turretPower) < 0.15) {
                turretPower = (turretPower > 0) ? 0.15 : -0.15;
            }
            telemetry.addLine("Fine tuning");
        }

        shooter.setTurretManualPower(turretPower);
        telemetry.addData("Turret Power", "%.3f", turretPower);
        telemetry.addData("Direction", turretPower != 0.0 ? (turretPower > 0 ? "RIGHT" : "LEFT") : "STOPPED");
    }

    private void handleLocked() {
        // Maintain lock on target
        if (!vision.hasTargetTag()) {
            currentState = TurretState.TARGET_MODE_SEARCHING;
            telemetry.addLine("Target lost - back to searching!");
            return;
        }

        double yawDegrees = vision.getTargetYaw();
        if (Double.isNaN(yawDegrees)) {
            yawDegrees = 0.0;
        }

        // Re-adjust if drifted
        if (Math.abs(yawDegrees) > YAW_LOCK_THRESHOLD) {
            currentState = TurretState.AUTO_AIM;
            telemetry.addLine("Drifted - re-aiming");
        } else {
            // Hold position
            shooter.holdTurretPosition();
            telemetry.addLine("*** LOCKED ON TARGET ***");
            telemetry.addData("Yaw Error", "%.2f deg", yawDegrees);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Mode", currentState);
        telemetry.addData("Alliance", vision.getAllianceColor());
        telemetry.addData("Turret Position", shooter.getTurretCurrentTicks());
        telemetry.addLine();

        telemetry.addData("Vision Active", vision.isActive());
        telemetry.addData("Any Target", vision.hasAnyTarget());
        telemetry.addData("Alliance Target", vision.hasTargetTag());

        if (vision.hasAnyTarget()) {
            telemetry.addData("Best Tag ID", vision.getBestTagId());
        }

        if (vision.hasTargetTag()) {
            double yaw = vision.getTargetYaw();
            double distance = vision.getTargetDistance();

            if (!Double.isNaN(yaw)) {
                telemetry.addData("Target Yaw", "%.2f deg", yaw);
            }
            if (distance >= 0) {
                telemetry.addData("Target Distance", "%.1f cm", distance);
            }
        }

        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  Right Stick X: Manual turret control");
        telemetry.addLine("  Right Trigger: Toggle target mode ON/OFF");
        telemetry.addLine("  Dpad Up: RED alliance (Tag 24)");
        telemetry.addLine("  Dpad Down: BLUE alliance (Tag 20)");

        telemetry.update();
    }
}
