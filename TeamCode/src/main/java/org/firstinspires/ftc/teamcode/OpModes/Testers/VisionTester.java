package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="[TEST] Vision Tester", group="Testers")
public class VisionTester extends LinearOpMode {

    private Vision vision;
    private Turret turret;
    private boolean isRedAlliance = true;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Vision...");
        telemetry.update();

        vision = new Vision();
        vision.init(hardwareMap);
        vision.start();
        vision.setAlliance(isRedAlliance);

        // Initialize turret with Vision-only constructor
        turret = new Turret(hardwareMap, vision);

        telemetry.addData("Status", "Ready!");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addData("DPad Up", "Switch to RED Alliance");
        telemetry.addData("DPad Down", "Switch to BLUE Alliance");
        telemetry.addLine();
        telemetry.addData("Turret", "Auto-aim ENABLED (tracks AprilTag)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Alliance switching
            if (gamepad1.dpad_up && !prevDpadUp) {
                isRedAlliance = true;
                vision.setAlliance(true);
            }
            if (gamepad1.dpad_down && !prevDpadDown) {
                isRedAlliance = false;
                vision.setAlliance(false);
            }

            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;

            // Turret auto-aim (tracks AprilTag using Vision yaw)
            turret.autoAim();

            // Display telemetry
            displayTelemetry();

            telemetry.update();
            sleep(50);
        }

        turret.stop();
        vision.stop();
    }

    private double calculateHoodPosition(double distance) {
        // Диапазоны: 30-150 см → 0.0-0.4, 150-300 см → 0.4-0.5
        final double MIN_DISTANCE = 30.0;
        final double MID_DISTANCE = 150.0;
        final double MAX_DISTANCE = 300.0;

        if (distance <= MIN_DISTANCE) {
            return 0.0; // Минимум
        } else if (distance <= MID_DISTANCE) {
            // 30-150: интерполяция 0.0 → 0.4
            double ratio = (distance - MIN_DISTANCE) / (MID_DISTANCE - MIN_DISTANCE);
            return ratio * 0.4;
        } else if (distance <= MAX_DISTANCE) {
            // 150-300: интерполяция 0.4 → 0.5
            double ratio = (distance - MID_DISTANCE) / (MAX_DISTANCE - MID_DISTANCE);
            return 0.4 + (ratio * 0.1);
        } else {
            return 0.5; // Максимум
        }
    }

    private void displayTelemetry() {
        telemetry.addLine("=== VISION TESTER ===");
        telemetry.addData("Alliance", vision.getAllianceColor());
        telemetry.addData("Target Tag ID", vision.getTargetTagId());
        telemetry.addLine();

        // Target Tag Info
        telemetry.addLine("--- TARGET TAG ---");
        boolean hasTarget = vision.hasTargetTag();
        telemetry.addData("Target Visible", hasTarget ? "YES" : "NO");

        if (hasTarget) {
            double distance = vision.getTargetDistance();
            double yaw = vision.getTargetYaw();

            telemetry.addData("Distance (cm)", "%.2f", distance);
            telemetry.addData("Yaw (degrees)", "%.2f", yaw);

            // Hood calculation based on distance (servo position 0.0-0.5)
            double hoodServoPos = calculateHoodPosition(distance);
            telemetry.addData("Hood Servo Position", "%.3f", hoodServoPos);

            // Range indicator
            String rangeIndicator;
            if (distance < 30) {
                rangeIndicator = "TOO CLOSE";
            } else if (distance <= 150) {
                rangeIndicator = "CLOSE-MID (30-150cm)";
            } else if (distance <= 300) {
                rangeIndicator = "MID-FAR (150-300cm)";
            } else {
                rangeIndicator = "OUT OF RANGE";
            }
            telemetry.addData("Range", rangeIndicator);
        } else {
            telemetry.addData("Distance", "---");
            telemetry.addData("Yaw", "---");
            telemetry.addData("Suggested Hood", "---");
        }

        telemetry.addLine();

        // Turret section
        telemetry.addLine("--- TURRET ---");
        telemetry.addData("Current Angle", "%.1f°", turret.getCurrentAngle());
        telemetry.addData("Target Angle", "%.1f°", turret.getTargetAngle());
        telemetry.addData("At Target", turret.atTarget() ? "YES" : "NO");
        telemetry.addData("Motor Power", "%.3f", turret.getMotorPower());

        // Tracking status
        if (vision.hasTargetTag()) {
            telemetry.addData("Status", "TRACKING TAG");
        } else {
            telemetry.addData("Status", "NO TARGET");
        }

        telemetry.addLine();

        // All detected tags
        telemetry.addLine("--- ALL DETECTED TAGS ---");
        AprilTagDetection bestTag = vision.getBestTag();
        if (bestTag != null) {
            telemetry.addData("Best Tag ID", bestTag.id);
            telemetry.addData("Best Tag Range", "%.2f cm", bestTag.ftcPose.range * 2.54);
        } else {
            telemetry.addData("All Tags", "None detected");
        }

        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("DPad Up", "RED Alliance");
        telemetry.addData("DPad Down", "BLUE Alliance");
    }
}
