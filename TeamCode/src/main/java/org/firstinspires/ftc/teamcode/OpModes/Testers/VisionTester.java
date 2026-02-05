package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="[TEST] Vision Tester", group="Testers")
public class VisionTester extends LinearOpMode {

    private Vision vision;
    private Turret turret;
    private Follower follower; // Для Kalman Filter sensor fusion
    private DriveTrain driveTrain;
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

        // Initialize Follower for Kalman Filter sensor fusion
        telemetry.addData("Status", "Initializing Odometry...");
        telemetry.update();

        follower = Constants.createFollower(hardwareMap);
        follower.update(); // Critical first update

        // Set starting pose (Blue alliance position as default)
        Pose startPose = new Pose(50, 95, Math.toRadians(270));
        follower.setStartingPose(startPose);

        // Initialize DriveTrain
        driveTrain = new DriveTrain(hardwareMap, telemetry);

        // Initialize turret with Vision + Follower for Kalman Filter
        turret = new Turret(hardwareMap, vision, follower);

        // Set goal pose (basket) for Kalman Filter tracking
        // This automatically initializes the Kalman Filter in Turret
        Pose goalPose = new Pose(12, 136, 270); // Blue basket position
        turret.setGoalPose(goalPose);

        telemetry.addData("Status", "Ready!");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addData("GP1 Left Stick", "Drive (X/Y)");
        telemetry.addData("GP1 Right Stick", "Rotate");
        telemetry.addData("GP1 DPad Up", "Switch to RED Alliance");
        telemetry.addData("GP1 DPad Down", "Switch to BLUE Alliance");
        telemetry.addLine();
        telemetry.addData("Turret", "Auto-aim ENABLED (Kalman Filter)");
        telemetry.addData("Kalman Filter", "ENABLED for sensor fusion");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update odometry FIRST every loop
            follower.update();

            // DriveTrain control (gamepad1)
            driveTrain.drive(gamepad1, gamepad2, telemetry);

            // Alliance switching
            if (gamepad1.dpad_up && !prevDpadUp) {
                isRedAlliance = true;
                vision.setAlliance(true);
                // Update goal pose for RED alliance
                Pose redGoalPose = new Pose(12, 12, 90); // Red basket position
                turret.setGoalPose(redGoalPose);
            }
            if (gamepad1.dpad_down && !prevDpadDown) {
                isRedAlliance = false;
                vision.setAlliance(false);
                // Update goal pose for BLUE alliance
                Pose blueGoalPose = new Pose(12, 136, 270); // Blue basket position
                turret.setGoalPose(blueGoalPose);
            }

            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;

            // Turret auto-aim (tracks AprilTag using Kalman Filter sensor fusion)
            turret.autoAim();

            // Display telemetry
            displayTelemetry();

            telemetry.update();
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
            double tx = vision.getTargetYaw();        // Horizontal offset
            double ty = vision.getTargetPitch();      // Vertical offset

            telemetry.addData("Distance (cm)", String.format("%.2f", distance));
            telemetry.addData("tx (H offset)", String.format("%.2f°", tx));
            telemetry.addData("ty (V offset)", String.format("%.2f°", ty));

            // Hood calculation based on distance (servo position 0.0-0.5)
            double hoodServoPos = calculateHoodPosition(distance);
            telemetry.addData("Hood Servo", String.format("%.3f", hoodServoPos));

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
            telemetry.addData("tx (H offset)", "---");
            telemetry.addData("ty (V offset)", "---");
            telemetry.addData("Hood Servo", "---");
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

        // Debug: Limelight raw data (tx, ty)
        telemetry.addLine();
        telemetry.addLine("--- DEBUG: LIMELIGHT RAW DATA ---");
        telemetry.addData("Data", vision.getDebugLimelightData());

        // Kalman Filter section
        telemetry.addLine();
        telemetry.addLine("--- KALMAN FILTER ---");
        telemetry.addData("Status", turret.isKalmanEnabled() ? "ENABLED ✓" : "DISABLED");

        if (turret.isKalmanEnabled()) {
            // Odometry position
            Pose currentPose = follower.getPose();
            telemetry.addData("Robot X (Odo)", String.format("%.1f cm", currentPose.getX()));
            telemetry.addData("Robot Y (Odo)", String.format("%.1f cm", currentPose.getY()));
            telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(currentPose.getHeading())));

            // Filtered goal position
            telemetry.addData("Filtered Goal X", String.format("%.2f cm", turret.debugFilteredX));
            telemetry.addData("Filtered Goal Y", String.format("%.2f cm", turret.debugFilteredY));

            // Innovation (measurement - prediction difference)
            telemetry.addData("Innovation X", String.format("%.2f cm", turret.debugInnovation[0]));
            telemetry.addData("Innovation Y", String.format("%.2f cm", turret.debugInnovation[1]));

            // Innovation magnitude check
            double innovationMagnitude = Math.sqrt(
                turret.debugInnovation[0] * turret.debugInnovation[0] +
                turret.debugInnovation[1] * turret.debugInnovation[1]
            );
            telemetry.addData("Innovation Mag", String.format("%.2f cm", innovationMagnitude));

            if (innovationMagnitude > 10.0) {
                telemetry.addLine("⚠️ Large innovation - measurement differs from prediction!");
            }

            // Outlier rejection count
            telemetry.addData("Outliers Rejected", turret.debugOutlierCount);

            // Distance to goal
            telemetry.addData("Distance to Goal", String.format("%.1f cm", turret.getDistanceToGoal()));
        }

        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("GP1 Left/Right Stick", "Drive robot");
        telemetry.addData("GP1 DPad Up/Down", "Switch alliance");
    }
}
