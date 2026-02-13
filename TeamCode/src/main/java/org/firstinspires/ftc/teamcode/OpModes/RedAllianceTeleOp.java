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
    public static boolean ENABLE_KALMAN = false;

    private Robot robot;

    // Mode selection
    private TeleOpMode selectedMode = TeleOpMode.NORMAL;  // Default
    private boolean modeConfirmed = false;
    private boolean prevRightBumper = false;

    @Override
    public void runOpMode() {
        // RED Alliance
        boolean isRedAlliance = true;

        // Mode selection during init phase
        while (!isStarted() && !isStopRequested()) {
            // Allow mode switching with dpad (always, even after confirmation)
            if (gamepad1.dpad_up) {
                selectedMode = TeleOpMode.NORMAL;
            } else if (gamepad1.dpad_down) {
                selectedMode = TeleOpMode.EMERGENCY;
            }

            // Confirm selection with right bumper
            if (gamepad1.right_bumper && !prevRightBumper) {
                modeConfirmed = true;
            }
            prevRightBumper = gamepad1.right_bumper;

            // Display
            telemetry.addLine("=== RED ALLIANCE TELEOP ===");
            telemetry.addLine();
            telemetry.addLine("=== SELECT MODE ===");
            telemetry.addData("Current", selectedMode == TeleOpMode.NORMAL ? ">>> NORMAL <<<" : "NORMAL");
            telemetry.addData("", selectedMode == TeleOpMode.EMERGENCY ? ">>> EMERGENCY <<<" : "EMERGENCY");
            telemetry.addLine();

            if (selectedMode == TeleOpMode.NORMAL) {
                telemetry.addLine("NORMAL Mode:");
                telemetry.addLine("- Turret auto-aims on startup");
                telemetry.addLine("- Standard operation");
            } else {
                telemetry.addLine("EMERGENCY Mode:");
                telemetry.addLine("- NO auto-aim on startup");
                telemetry.addLine("- Manual turret → position reset → enable auto-aim");
            }

            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addData("Dpad UP/DOWN", "Select mode");
            telemetry.addData("Right Bumper", "Confirm (optional)");
            telemetry.addData("Status", modeConfirmed ? "CONFIRMED ✓" : "Press START to begin");
            telemetry.addLine();
            telemetry.addData("Target AprilTag", "24 (RED)");
            telemetry.update();
        }

        waitForStart();

        // Initialize robot with selected mode
        robot = new Robot(hardwareMap, telemetry, isRedAlliance, selectedMode);

        // Check if Auto set a position - if so, use it. Otherwise use default.
        org.firstinspires.ftc.teamcode.SubSystems.Localizer localizer =
            org.firstinspires.ftc.teamcode.SubSystems.Localizer.getInstance();

        double lastX = localizer.getX();
        double lastY = localizer.getY();
        double lastHeading = localizer.getHeading();

        Pose startPose;
        if (Math.abs(lastX) > 1.0 || Math.abs(lastY) > 1.0) {
            // Auto ran and set position - use it
            startPose = new Pose(lastX, lastY, Math.toRadians(lastHeading));
            telemetry.addLine("✓ Using position from Auto");
            telemetry.addData("Auto End Pos", "(%.1f, %.1f, %.0f°)", lastX, lastY, lastHeading);
        } else {
            // No Auto ran - use default Red start
            startPose = new Pose(119, 73, Math.toRadians(270));
            telemetry.addLine("Using default Red start position");
        }

        robot.follower.setStartingPose(startPose);

        // Synchronize Localizer with Follower (in case we used default)
        Pose syncPose = robot.follower.getPose();
        localizer.setPosition(
            syncPose.getX(),
            syncPose.getY(),
            Math.toDegrees(syncPose.getHeading())
        );


        // Set goal (basket) for turret auto-aim
        Pose redGoalPose = new Pose(130, 135, Math.toRadians(270));
        robot.turret.setGoalPose(redGoalPose);

        // Enable Kalman Filter for sensor fusion
        robot.turret.setKalmanEnabled(ENABLE_KALMAN);

        robot.start();

        while (opModeIsActive()) {
            // Position reset on dpad_up (gamepad1)
            if (gamepad1.dpad_up) {
                // Reset to Red alliance preset position
                Pose resetPose = new Pose(8.282087447108605, 8.153485190409029, Math.toRadians(180));
                robot.follower.setPose(resetPose);

                // Synchronize Localizer
                org.firstinspires.ftc.teamcode.SubSystems.Localizer.getInstance().setPosition(
                    resetPose.getX(),
                    resetPose.getY(),
                    Math.toDegrees(resetPose.getHeading())
                );

                // Force turret to auto-aim mode and point to goal
                robot.turretController.enableAutoAim();
                robot.turret.autoAim();

                telemetry.addLine("⚠️ POSITION RESET TO (8.28, 8.15)");
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
        // Mode indicator at top
        telemetry.addLine("=== MODE ===");
        telemetry.addData("TeleOp Mode", selectedMode);
        if (selectedMode == TeleOpMode.EMERGENCY) {
            telemetry.addLine("⚠️ EMERGENCY: Turret manual only");
        }
        telemetry.addLine();

        // Odometry
        Pose currentPose = robot.follower.getPose();
        telemetry.addLine("=== ODOMETRY (RED) ===");
        telemetry.addData("Robot X", "%.2f cm", currentPose.getX());
        telemetry.addData("Robot Y", "%.2f cm", currentPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));

        // Vision
        telemetry.addLine();
//        telemetry.addLine("=== VISION (RED) ===");
//        telemetry.addData("Alliance", robot.vision.getAllianceColor());
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
//        telemetry.addLine();
//        telemetry.addLine("=== TURRET ===");
//        telemetry.addData("Current Angle", "%.1f°", robot.turret.getCurrentAngle());
//        telemetry.addData("Target Angle", "%.1f°", robot.turret.getTargetAngle());
//
//        String turretMode;
//        if (robot.turret.atTarget()) {
//            turretMode = "AT TARGET";
//        } else {
//            turretMode = "MOVING";
//        }
//        telemetry.addData("Mode", turretMode);
//        telemetry.addData("Auto Aim", robot.turretController.isAutoAimEnabled() ? "ON" : "MANUAL");
//        telemetry.addData("Centered", robot.turret.isCentered() ? "YES" : "NO");

        // Kalman Filter
//        telemetry.addLine();
//        telemetry.addLine("=== KALMAN FILTER ===");
//        telemetry.addData("Enabled", robot.turret.isKalmanEnabled() ? "YES ✓" : "NO (Legacy EMA)");
//        if (robot.turret.isKalmanEnabled()) {
//            telemetry.addData("Filtered Target X", "%.2f cm", robot.turret.debugFilteredX);
//            telemetry.addData("Filtered Target Y", "%.2f cm", robot.turret.debugFilteredY);
//            telemetry.addData("Innovation X", "%.2f cm", robot.turret.debugInnovation[0]);
//            telemetry.addData("Innovation Y", "%.2f cm", robot.turret.debugInnovation[1]);
//            telemetry.addData("Outliers Rejected", robot.turret.debugOutlierCount);
//
//            // Innovation magnitude check
//            double innovationMagnitude = Math.sqrt(
//                robot.turret.debugInnovation[0] * robot.turret.debugInnovation[0] +
//                robot.turret.debugInnovation[1] * robot.turret.debugInnovation[1]
//            );
//            if (innovationMagnitude > 10.0) {
//                telemetry.addLine("⚠️ Large innovation!");
//            }
//        }

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

//        telemetry.addData("State", robot.shooterController.getCurrentState());
//        telemetry.addData("Is Shooting", robot.shooterController.isShooting() ? "YES" : "NO");
//        telemetry.addData("Sample Count", "%d / 3", robot.shooter.getSampleCount());
        telemetry.addData("Hood Servo Position", "%.2f", robot.shooter.getHoodServoPosition());
//        telemetry.addData("ShooterStop Position", "%.2f", robot.shooter.getStopPosition());
//        telemetry.addData("IntakeStop Position", "%.2f", robot.shooter.getIntakeStopPosition());
//        telemetry.addData("Target Velocity", "%.0f ticks/sec", robot.shooter.getTargetVelocity());
        telemetry.addData("Current Velocity", "%.0f ticks/sec", robot.shooter.getCurrentVelocity());

        // Controls
//        telemetry.addLine();
//        telemetry.addLine("=== CONTROLS ===");
//        telemetry.addData("GP1 Dpad Left", "TURRET CALIBRATION (rotate left, release=reset encoder)");
//        telemetry.addData("GP1 Dpad Right", "TURRET CALIBRATION (rotate right, release=reset encoder)");
//        telemetry.addData("GP1 Dpad Down", "RESET POSITION (104, 135)");
//        telemetry.addData("GP2 Right Bumper", "Start Shoot");
//        telemetry.addData("GP2 Dpad Up", "Manual Open ShooterStop");
//        telemetry.addData("GP2 Dpad Down", "Manual Close ShooterStop");
//        telemetry.addData("GP2 Right Stick X", "Manual Turret (holds position)");
//        telemetry.addData("GP2 Left Bumper", "Re-enable Auto-Aim");
//        telemetry.addData("GP2 Options", "RESET ALL");
    }
}
