package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

@TeleOp(name="BLUE Alliance TeleOp", group="TeleOp")
public class BlueAllianceTeleOp extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {
        // BLUE Alliance
        boolean isRedAlliance = false;

        // Инициализация робота
        telemetry.addData("Status", "Initializing...");
        telemetry.addData("Alliance", "BLUE");
        telemetry.addData("Target AprilTag ID", 20);
        telemetry.update();

        robot = new Robot(hardwareMap, telemetry, isRedAlliance);

        // Set starting pose for Blue alliance
        Pose blueStartPose = new Pose(50, 95, Math.toRadians(270));
        robot.follower.setStartingPose(blueStartPose);

        // Set goal (basket) for turret auto-aim
        Pose blueGoalPose = new Pose(136, 11, 270);
        robot.turret.setGoalPose(blueGoalPose);

        telemetry.addData("Status", "Ready to start!");
        telemetry.addData("Start Pose", "(%.1f, %.1f)", blueStartPose.getX(), blueStartPose.getY());
        telemetry.addData("Goal Pose", "(%.1f, %.1f)", blueGoalPose.getX(), blueGoalPose.getY());
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
        telemetry.addLine("=== VISION (BLUE) ===");
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

        // Shooter
        telemetry.addLine();
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("State", robot.shooterController.getCurrentState());
        telemetry.addData("Is Shooting", robot.shooterController.isShooting() ? "YES" : "NO");
        telemetry.addData("Hood Position", robot.shooter.getCurrentHoodPosition());

        // Controls
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("GP2 Right Bumper", "Start Shoot");
        telemetry.addData("GP2 Right Stick X", "Manual Turret");
        telemetry.addData("GP2 Options", "RESET ALL");
    }
}