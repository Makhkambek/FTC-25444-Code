package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;
// import org.firstinspires.ftc.teamcode.SubSystems.Vision;

@TeleOp(name="[TEST] Turret Tester", group="Testers")
public class TurretTester extends LinearOpMode {

    private Turret turret;
    // private Vision vision;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;

    private enum TestMode {
        MANUAL,     // Ручное управление с PID
        AUTO        // Auto-aim / scanning (с Vision)
    }

    private TestMode currentMode = TestMode.MANUAL;
    private double targetAngle = 0.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Turret...");
        telemetry.update();

        // vision = new Vision();
        // vision.init(hardwareMap);
        // vision.start();
        // vision.setAlliance(true); // RED

        turret = new Turret(hardwareMap, null); // Pass null for vision in testing

        telemetry.addData("Status", "Ready!");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        // Reset encoder
        turret.resetEncoder();

        while (opModeIsActive()) {
            handleControls();
            displayTelemetry();
            telemetry.update();
        }

        turret.stop();
        // vision.stop();
    }

    private void handleControls() {
        if (currentMode == TestMode.MANUAL) {
            // === MANUAL MODE ===

            // Right stick for smooth manual control with PID
            double stickInput = -gamepad1.right_stick_x;
            if (Math.abs(stickInput) > 0.1) {
                turret.syncManualTarget();
                turret.manualControl(stickInput);
            }

            // Preset positions
            if (gamepad1.dpad_left && !prevA) {
                targetAngle = -90.0; // Max left
            }
            if (gamepad1.dpad_up && !prevB) {
                targetAngle = 0.0; // Center
            }
            if (gamepad1.dpad_right && !prevX) {
                targetAngle = 90.0; // Max right
            }

            // Apply target if set by dpad
            if (gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right) {
                turret.syncManualTarget();
                // Move towards target using manual control
            }

            // Center command
            if (gamepad1.a && !prevA) {
                turret.returnToCenter();
            }

            // Stop
            if (gamepad1.b && !prevB) {
                turret.stop();
            }

        } else {
            // === AUTO MODE ===
            // Auto-aim with vision (uncomment when ready)
            // turret.autoAim();

            // For now, just show what would happen
            telemetry.addLine("AUTO MODE - Vision integration needed");
            telemetry.addLine("Uncomment vision code to enable");
        }

        // Mode switching
        if (gamepad1.y && !prevY) {
            currentMode = (currentMode == TestMode.MANUAL) ? TestMode.AUTO : TestMode.MANUAL;
            if (currentMode == TestMode.AUTO) {
                // Prepare for auto mode
                turret.syncManualTarget();
            }
        }

        // Save button states
        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevY = gamepad1.y;
    }

    private void displayTelemetry() {
        telemetry.addLine("=== TURRET TESTER ===");
        telemetry.addData("Mode", currentMode);
        telemetry.addLine();

        // Status
        telemetry.addLine("--- STATUS ---");
        telemetry.addData("Current Angle", "%.2f°", turret.getCurrentAngle());
        telemetry.addData("Is Centered", turret.isCentered() ? "YES" : "NO");
        telemetry.addData("Is Resetting", turret.isResettingToCenter() ? "YES" : "NO");

        telemetry.addLine();

        if (currentMode == TestMode.MANUAL) {
            telemetry.addLine("=== MANUAL MODE ===");
            telemetry.addData("Right Stick X", "Manual Control (PID)");
            telemetry.addData("DPad Left", "Go to -90°");
            telemetry.addData("DPad Up", "Go to 0° (Center)");
            telemetry.addData("DPad Right", "Go to +90°");
            telemetry.addData("A", "Return to Center");
            telemetry.addData("B", "STOP");
            telemetry.addData("Y", "Switch to AUTO Mode");
            telemetry.addLine();
            telemetry.addLine("Angle limits: -90° to +90°");

        } else {
            telemetry.addLine("=== AUTO MODE ===");
            telemetry.addData("Y", "Switch to MANUAL Mode");
            telemetry.addLine();

            // Vision info (uncomment when ready)
            // telemetry.addLine("--- VISION ---");
            // telemetry.addData("Target Visible", vision.hasTargetTag() ? "YES" : "NO");
            // if (vision.hasTargetTag()) {
            //     telemetry.addData("Target Yaw", "%.2f°", vision.getTargetYaw());
            // }
            // telemetry.addLine();
            // telemetry.addData("Turret Status", turret.isTracking() ? "TRACKING" : "SCANNING");

            telemetry.addLine();
            telemetry.addLine("Vision integration disabled for testing");
            telemetry.addLine("Uncomment code to enable auto-aim");
        }
    }
}
