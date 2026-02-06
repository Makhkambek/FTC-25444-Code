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
        Pose redStartPose = new Pose(103, 136, Math.toRadians(270));
        robot.follower.setStartingPose(redStartPose);

        // Set goal (basket) for turret auto-aim
        Pose redGoalPose = new Pose(136, 104, 270);
        robot.turret.setGoalPose(redGoalPose);

        // Enable Kalman Filter for sensor fusion
        robot.turret.setKalmanEnabled(ENABLE_KALMAN);

        telemetry.addData("Status", "Ready to start!");
        telemetry.addData("Start Pose", "(%.1f, %.1f)", redStartPose.getX(), redStartPose.getY());
        telemetry.addData("Goal Pose", "(%.1f, %.1f)", redGoalPose.getX(), redGoalPose.getY());
        telemetry.update();

        waitForStart();

        robot.start();

        while (opModeIsActive()) {
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

        double distance = robot.vision.getTargetDistance();
        if (distance > 0) {
            telemetry.addData("Distance (cm)", "%.1f", distance);
        } else {
            telemetry.addData("Distance", "---");
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
        telemetry.addData("State", robot.shooterController.getCurrentState());
        telemetry.addData("Is Shooting", robot.shooterController.isShooting() ? "YES" : "NO");
        telemetry.addData("Hood Position", robot.shooter.getCurrentHoodPosition());
        telemetry.addData("ShooterStop Position", "%.2f", robot.shooter.getStopPosition());
        telemetry.addData("IntakeStop Position", "%.2f", robot.shooter.getIntakeStopPosition());

        // Controls
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("GP2 Right Bumper", "Start Shoot");
        telemetry.addData("GP2 Right Stick X", "Manual Turret");
        telemetry.addData("GP2 Options", "RESET ALL");
    }
}
