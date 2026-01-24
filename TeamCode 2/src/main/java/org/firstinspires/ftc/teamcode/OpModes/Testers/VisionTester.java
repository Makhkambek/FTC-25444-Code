package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="[TEST] Vision Tester", group="Testers")
public class VisionTester extends LinearOpMode {

    private Vision vision;
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

        telemetry.addData("Status", "Ready!");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addData("DPad Up", "Switch to RED Alliance");
        telemetry.addData("DPad Down", "Switch to BLUE Alliance");
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

            // Display telemetry
            displayTelemetry();

            telemetry.update();
            sleep(50);
        }

        vision.stop();
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

            // Hood calculation based on distance
            String hoodPos;
            if (distance < 30) {
                hoodPos = "CLOSE";
            } else if (distance < 50) {
                hoodPos = "MIDDLE";
            } else if (distance < 100) {
                hoodPos = "FAR";
            } else {
                hoodPos = "OUT OF RANGE";
            }
            telemetry.addData("Suggested Hood", hoodPos);
        } else {
            telemetry.addData("Distance", "---");
            telemetry.addData("Yaw", "---");
            telemetry.addData("Suggested Hood", "---");
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
