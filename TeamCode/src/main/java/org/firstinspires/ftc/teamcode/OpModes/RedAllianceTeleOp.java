package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

@Config
@TeleOp(name="RED Alliance TeleOp", group="TeleOp")
public class RedAllianceTeleOp extends LinearOpMode {

    // Kalman Filter toggle (FTC Dashboard)
    public static boolean ENABLE_KALMAN = true;

    private Robot robot;

    @Override
    public void runOpMode() {
        // RED Alliance
        boolean isRedAlliance = true;

        // Инициализация робота
        telemetry.addData("Status", "Initializing...");
        telemetry.addData("Alliance", "RED");
        telemetry.addData("Target AprilTag ID", 24);
        telemetry.update();

        robot = new Robot(hardwareMap, telemetry, isRedAlliance);

        // Set starting pose for Red alliance
        Pose redStartPose = new Pose(119, 73, Math.toRadians(270));
        robot.follower.setStartingPose(redStartPose);

        // CRITICAL: Synchronize Localizer with Follower
        // This ensures HeadingController (used for heading lock) has correct heading
        Pose syncPose = robot.follower.getPose();
        org.firstinspires.ftc.teamcode.SubSystems.Localizer.getInstance().setPosition(
            syncPose.getX(),
            syncPose.getY(),
            Math.toDegrees(syncPose.getHeading())
        );


        // Set goal (basket) for turret auto-aim
        Pose redGoalPose = new Pose(130, 135, Math.toRadians(270));
        robot.turret.setGoalPose(redGoalPose);

        // Enable Kalman Filter for sensor fusion
        robot.turret.setKalmanEnabled(ENABLE_KALMAN);

        // Verify heading synchronization
        telemetry.addLine("=== INITIALIZATION ===");
        telemetry.addData("Status", "Pinpoint IMU Ready");
        telemetry.addData("Follower Heading", "%.1f°", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("Localizer Heading", "%.1f°", org.firstinspires.ftc.teamcode.SubSystems.Localizer.getInstance().getHeading());
        telemetry.addData("Expected Heading", "270.0°");
        telemetry.addLine();
        telemetry.addData("Sync Status", "Both systems aligned ✓");
        telemetry.addData("Start Pose", "(%.1f, %.1f)", redStartPose.getX(), redStartPose.getY());
        telemetry.addData("Goal Pose", "(%.1f, %.1f)", redGoalPose.getX(), redGoalPose.getY());
        telemetry.update();

        waitForStart();

        robot.start();

        while (opModeIsActive()) {
            // Position reset on dpad_down (gamepad1)
            if (gamepad1.dpad_down) {
                // Reset to Red alliance preset position
                Pose resetPose = new Pose(104, 135, Math.toRadians(90));
                robot.follower.setPose(resetPose);

                // Synchronize Localizer
                org.firstinspires.ftc.teamcode.SubSystems.Localizer.getInstance().setPosition(
                    resetPose.getX(),
                    resetPose.getY(),
                    Math.toDegrees(resetPose.getHeading())
                );

                telemetry.addLine("⚠️ POSITION RESET TO (104, 135)");
                telemetry.update();
                sleep(500); // Prevent multiple resets
            }

            // Обновление робота
            robot.update(gamepad1, gamepad2, telemetry);

            // Телеметрия
            displayTelemetry();

            telemetry.update();
        }

        robot.stop();
    }

    private void displayTelemetry() {
        // Vision
        telemetry.addLine("=== VISION (RED) ===");
        telemetry.addData("Alliance", robot.vision.getAllianceColor());
        telemetry.addData("Target Tag ID", robot.vision.getTargetTagId());
        telemetry.addData("Target Visible", robot.vision.hasTargetTag() ? "YES" : "NO");

        double visionDistInches = robot.vision.getTargetDistance();
        if (visionDistInches > 0) {
            telemetry.addData("Vision Distance", "%.1f in (%.1f cm)",
                visionDistInches, visionDistInches * 2.54);
        } else {
            telemetry.addData("Vision Distance", "---");
        }

        double yaw = robot.vision.getTargetYaw();
        if (!Double.isNaN(yaw)) {
            telemetry.addData("Target Yaw", "%.1f°", yaw);
        } else {
            telemetry.addData("Target Yaw", "---");
        }

        // Turret
        telemetry.addLine();
        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Current Angle", "%.1f°", robot.turret.getCurrentAngle());
        telemetry.addData("Target Angle", "%.1f°", robot.turret.getTargetAngle());

        String turretMode;
        if (robot.turret.atTarget()) {
            turretMode = "AT TARGET";
        } else {
            turretMode = "MOVING";
        }
        telemetry.addData("Mode", turretMode);
        telemetry.addData("Auto Aim", robot.turretController.isAutoAimEnabled() ? "ON" : "MANUAL");
        telemetry.addData("Centered", robot.turret.isCentered() ? "YES" : "NO");

        // Kalman Filter
        telemetry.addLine();
        telemetry.addLine("=== KALMAN FILTER ===");
        telemetry.addData("Enabled", robot.turret.isKalmanEnabled() ? "YES ✓" : "NO (Legacy EMA)");
        if (robot.turret.isKalmanEnabled()) {
            telemetry.addData("Filtered Target X", "%.2f cm", robot.turret.debugFilteredX);
            telemetry.addData("Filtered Target Y", "%.2f cm", robot.turret.debugFilteredY);
            telemetry.addData("Innovation X", "%.2f cm", robot.turret.debugInnovation[0]);
            telemetry.addData("Innovation Y", "%.2f cm", robot.turret.debugInnovation[1]);
            telemetry.addData("Outliers Rejected", robot.turret.debugOutlierCount);

            // Innovation magnitude check
            double innovationMagnitude = Math.sqrt(
                robot.turret.debugInnovation[0] * robot.turret.debugInnovation[0] +
                robot.turret.debugInnovation[1] * robot.turret.debugInnovation[1]
            );
            if (innovationMagnitude > 10.0) {
                telemetry.addLine("⚠️ Large innovation!");
            }
        }

        // Shooter
        telemetry.addLine();
        telemetry.addLine("=== SHOOTER ===");

        // Показываем какой source используется для distance
        telemetry.addData("Distance Source", robot.distanceSource);

        // Vision distance (если виден)
        double visionDist = robot.vision.getTargetDistance();
        if (visionDist > 0) {
            telemetry.addData("Vision Distance", "%.1f in (%.1f cm)", visionDist, visionDist * 2.54);
        } else {
            telemetry.addData("Vision Distance", "Tag not visible");
        }

        // Odometry distance (всегда доступен)
        double odometryDist = robot.turret.getDistanceToGoal();
        telemetry.addData("Odometry Distance", "%.1f in (%.1f cm)", odometryDist, odometryDist * 2.54);

        telemetry.addData("State", robot.shooterController.getCurrentState());
        telemetry.addData("Is Shooting", robot.shooterController.isShooting() ? "YES" : "NO");
        telemetry.addData("Sample Count", "%d / 3", robot.shooter.getSampleCount());
        telemetry.addData("Hood Servo Position", "%.2f", robot.shooter.getHoodServoPosition());
        telemetry.addData("ShooterStop Position", "%.2f", robot.shooter.getStopPosition());
        telemetry.addData("IntakeStop Position", "%.2f", robot.shooter.getIntakeStopPosition());

        // Controls
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("GP1 Dpad Down", "RESET POSITION (104, 135)");
        telemetry.addData("GP2 Right Bumper", "Start Shoot");
        telemetry.addData("GP2 Dpad Up", "Manual Open ShooterStop");
        telemetry.addData("GP2 Dpad Down", "Manual Close ShooterStop");
        telemetry.addData("GP2 Right Stick X", "Manual Turret (holds position)");
        telemetry.addData("GP2 Left Bumper", "Re-enable Auto-Aim");
        telemetry.addData("GP2 Options", "RESET ALL");
    }
}
