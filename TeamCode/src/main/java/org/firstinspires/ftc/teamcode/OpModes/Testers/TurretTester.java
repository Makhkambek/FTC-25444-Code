package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name="[TEST] Turret Tester", group="Testers")
public class TurretTester extends LinearOpMode {

    // PIDF коэффициенты для настройки через Dashboard (оптимизированы для плавности)
    public static double KP = 0.035;
    public static double KI = 0.0;
    public static double KD = 0.008;
    public static double KF = 0.0015; // Feedforward для преодоления трения

    // Test positions (в градусах) - теперь 270° диапазон (-135 до +135)
    public static double RED_POSITION = -90.0;  // Тест левая сторона
    public static double BLUE_POSITION = 90.0;  // Тест правая сторона

    private Turret turret;
    private Vision vision;
    private Follower follower; // Pedro Pathing с Pinpoint odometry
    private DriveTrain driveTrain;

    // Starting Poses - начальные позиции робота для разных альянсов
    private Pose blueStartPose;
    private Pose redStartPose;

    // Goal Poses - координаты корзин для разных альянсов
    private Pose blueGoalPose;
    private Pose redGoalPose;
    private Pose goalPose; // Текущая активная цель

    // Alliance selection
    private boolean isRedAlliance = false; // По умолчанию Blue

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false; // Для OVERRIDE MODE toggle
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false; // Для Alliance toggle

    private boolean visionTrackingEnabled = false; // Vision auto-aim

    // Режим управления
    private enum ControlMode {
        NORMAL,    // Обычный режим с PIDF
        OVERRIDE   // Аварийный режим - прямое управление мощностью
    }
    private ControlMode currentMode = ControlMode.NORMAL;

    @Override
    public void runOpMode() {
        // Настройка телеметрии для Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Инициализация Pedro Pathing Follower с Pinpoint odometry
        follower = Constants.createFollower(hardwareMap);

        // Starting Poses - начальные позиции робота для разных альянсов
        blueStartPose = new Pose(39.945, 136, Math.toRadians(270));  // Blue alliance start (X=horizontal, Y=vertical)  НЕ МЕНЯЙ
        redStartPose = new Pose(103, 136, Math.toRadians(270));          // Red alliance start (X=horizontal, Y=vertical) НЕ МЕНЯЙ

        // Goal Poses - координаты корзин для разных альянсов (ФИНАЛЬНЫЕ - НЕ МЕНЯТЬ!)
        blueGoalPose = new Pose(136, 40, 270);   // Blue alliance корзина
        redGoalPose = new Pose(136, 104, 270);   // Red alliance корзина

        // По умолчанию Blue alliance
        isRedAlliance = false;
        goalPose = blueGoalPose;

        // CRITICAL: Update Pinpoint ONCE before setting starting pose to initialize encoder data
        follower.update();

        follower.setStartingPose(blueStartPose);

        // Инициализация DriveTrain для правильного управления
        driveTrain = new DriveTrain(hardwareMap, (org.firstinspires.ftc.teamcode.SubSystems.Localizer) null);

        // Инициализация Vision
        vision = new Vision();
        vision.init(hardwareMap);
        vision.setAlliance(isRedAlliance); // BLUE alliance (tag 21) по умолчанию

        // Создаем Turret с Vision и Pedro Pathing Follower
        turret = new Turret(hardwareMap, vision, follower);

        // Устанавливаем начальный Goal Pose для турели
        turret.setGoalPose(goalPose);

        telemetry.addLine("=== TURRET TESTER WITH PEDRO PATHING ===");
        telemetry.addLine();
        telemetry.addLine("IMPORTANT: Watch 'DELTAS' section!");
        telemetry.addLine("Delta X/Y MUST change when you move!");
        telemetry.addLine();
        telemetry.addLine("DRIVETRAIN (Gamepad1):");
        telemetry.addLine("Left Stick: Movement");
        telemetry.addLine("Right Stick X: Rotation");
        telemetry.addLine("NOTE: Using Pedro Pathing Pinpoint odometry");
        telemetry.addLine();
        telemetry.addLine("TURRET NORMAL MODE (Gamepad1):");
        telemetry.addLine("Auto-aim: ALWAYS active (odometry tracking)");
        telemetry.addLine("Right Trigger (HOLD): Enable Vision tracking");
        telemetry.addLine("Dpad Up: CENTER (0°)");
        telemetry.addLine("Right Bumper: RED position (90°)");
        telemetry.addLine("Dpad Down: BLUE position (-90°)");
        telemetry.addLine("Left Bumper: Toggle Alliance (Red/Blue)");
        telemetry.addLine();
        telemetry.addLine("OVERRIDE MODE (X to toggle):");
        telemetry.addLine("Dpad Left: Move LEFT (direct power)");
        telemetry.addLine("Dpad Right: Move RIGHT (direct power)");
        telemetry.addLine();
        telemetry.addLine("COMMON:");
        telemetry.addLine("A: Stop turret");
        telemetry.addLine("B: Reset encoder");
        telemetry.addLine();
        telemetry.addLine("=== DEBUG INFO ===");
        telemetry.addLine("Using Pedro Pathing Pinpoint Odometry");
        telemetry.addLine("Auto-aim: ODOMETRY (Priority 1), Vision (Priority 2)");
        telemetry.addLine();
        telemetry.addData("Current Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addLine();
        telemetry.addLine("Starting Poses:");
        telemetry.addData("  Blue Start", "(%.1f, %.1f)", blueStartPose.getX(), blueStartPose.getY());
        telemetry.addData("  Red Start", "(%.1f, %.1f)", redStartPose.getX(), redStartPose.getY());
        telemetry.addLine();
        telemetry.addLine("Goal Poses:");
        telemetry.addData("  Blue Goal", "(%.1f, %.1f)", blueGoalPose.getX(), blueGoalPose.getY());
        telemetry.addData("  Red Goal", "(%.1f, %.1f)", redGoalPose.getX(), redGoalPose.getY());
        telemetry.addData("  Active Goal", "(%.1f, %.1f)", goalPose.getX(), goalPose.getY());
        telemetry.addLine();
        telemetry.addLine("FTC Dashboard настройки:");
        telemetry.addLine("- Tune PIDF coefficients (KP, KI, KD, KF)");
        telemetry.addLine("- Turret uses normalized angle calculation");
        telemetry.addLine("  Formula: targetDirection - robotHeading");
        telemetry.update();

        waitForStart();

        // Запускаем Vision
        vision.start();

        // Reset encoder
        turret.resetEncoder();

        while (opModeIsActive()) {
            // Обновляем Pedro Pathing Follower (одометрия)
            follower.update();

            // Manual drive control через DriveTrain (gamepad1)
            driveTrain.drive(gamepad1, gamepad2, telemetry);

            // Обновляем PIDF коэффициенты из Dashboard
            turret.setPIDF(KP, KI, KD, KF);

            // Turret control
            handleControls();
            displayTelemetry();
            telemetry.update();
        }

        turret.stop();
        vision.stop();
    }

    private void handleControls() {
        // Переключение альянса (Left Bumper)
        if (gamepad1.left_bumper && !prevLeftBumper) {
            isRedAlliance = !isRedAlliance;

            // Обновляем Goal Pose
            goalPose = isRedAlliance ? redGoalPose : blueGoalPose;
            turret.setGoalPose(goalPose);

            // Обновляем Starting Pose робота
            Pose startPose = isRedAlliance ? redStartPose : blueStartPose;
            follower.setPose(startPose);

            // Обновляем Vision alliance
            vision.setAlliance(isRedAlliance);
        }

        // Переключение режимов (X)
        if (gamepad1.x && !prevX) {
            currentMode = (currentMode == ControlMode.NORMAL) ? ControlMode.OVERRIDE : ControlMode.NORMAL;
            turret.syncManualTarget(); // Синхронизируем при переключении
        }

        if (currentMode == ControlMode.NORMAL) {
            // === NORMAL MODE - PIDF Control ===

            // ALWAYS update target angle based on odometry/vision
            turret.autoAim();

            // Track vision state for telemetry
            if (gamepad1.right_trigger > 0.5) {
                visionTrackingEnabled = true;
            } else {
                visionTrackingEnabled = false;
            }

            // Manual control via right stick REMOVED - only auto-aim now

            // Preset positions
            if (gamepad1.dpad_up && !prevDpadUp) {
                turret.returnToCenter(); // CENTER (0°)
            }

            if (gamepad1.right_bumper && !prevRightBumper) {
                turret.setTargetAngle(RED_POSITION); // RED (90°)
            }

            if (gamepad1.dpad_down && !prevDpadDown) {
                turret.setTargetAngle(BLUE_POSITION); // BLUE (-90°)
            }

        } else {
            // === OVERRIDE MODE - Direct Power Control ===

            double direction = 0;
            if (gamepad1.dpad_left) {
                direction = -1.0; // Влево
            } else if (gamepad1.dpad_right) {
                direction = 1.0; // Вправо
            }

            turret.manualOverride(direction);
        }

        // Stop (работает в обоих режимах)
        if (gamepad1.a && !prevA) {
            turret.stop();
        }

        // Reset encoder (работает в обоих режимах)
        if (gamepad1.b && !prevB) {
            turret.resetEncoder();
        }

        // Save button states
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevRightBumper = gamepad1.right_bumper;
        prevLeftBumper = gamepad1.left_bumper;
    }

    private void displayTelemetry() {
        telemetry.addLine("=== TURRET TESTER ===");
        telemetry.addLine();

        // Mode
        telemetry.addLine("--- MODE ---");
        String modeStr = (currentMode == ControlMode.NORMAL) ? "NORMAL (PIDF)" : "OVERRIDE (Direct)";
        telemetry.addData("Current Mode", modeStr);
        telemetry.addData("Switch Mode", "Press X");
        telemetry.addLine();

        // Position
        telemetry.addLine("--- POSITION ---");
        telemetry.addData("Current Angle", "%.1f°", turret.getCurrentAngle());
        telemetry.addData("Target Angle", "%.1f°", turret.getTargetAngle());
        telemetry.addData("Current Ticks", "%.0f", turret.getCurrentPosition());
        telemetry.addData("Target Ticks", "%.0f", turret.getTargetPosition());
        telemetry.addData("Error", "%.0f ticks",
            turret.getTargetPosition() - turret.getCurrentPosition());
        telemetry.addData("At Target", turret.atTarget() ? "YES" : "NO");
        telemetry.addLine();

        // Motor Power
        telemetry.addLine("--- MOTOR ---");
        telemetry.addData("Motor Power", "%.3f", turret.getMotorPower());
        telemetry.addLine();

        // PIDF коэффициенты (только в NORMAL режиме)
        if (currentMode == ControlMode.NORMAL) {
            telemetry.addLine("--- PIDF (Dashboard) ---");
            telemetry.addData("kP", "%.4f", KP);
            telemetry.addData("kI", "%.4f", KI);
            telemetry.addData("kD", "%.4f", KD);
            telemetry.addData("kF", "%.4f", KF);
            telemetry.addLine();
        }

        // Odometry (только в NORMAL режиме)
        if (currentMode == ControlMode.NORMAL) {
            telemetry.addLine("--- PEDRO PATHING ODOMETRY DEBUG ---");

            Pose currentPose = follower.getPose();
            double robotX = currentPose.getX();
            double robotY = currentPose.getY();
            double robotHeadingRad = currentPose.getHeading();

            // Конвертируем heading в стандартный диапазон [0°, 360°]
            double robotHeading = Math.toDegrees(robotHeadingRad);
            while (robotHeading < 0) robotHeading += 360;
            while (robotHeading >= 360) robotHeading -= 360;

            double goalX = goalPose.getX();
            double goalY = goalPose.getY();

            double deltaX = goalX - robotX;
            double deltaY = goalY - robotY;
            double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            telemetry.addLine("=== COORDINATES ===");
            telemetry.addData("Robot X", "%.2f cm", robotX);
            telemetry.addData("Robot Y", "%.2f cm", robotY);
            telemetry.addData("Robot Heading", "%.1f° (0-360)", robotHeading);
            telemetry.addData("  (raw from Pose)", "%.1f rad", robotHeadingRad);
            telemetry.addLine();
            telemetry.addData("Goal X", "%.2f cm", goalX);
            telemetry.addData("Goal Y", "%.2f cm", goalY);
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addLine();

            telemetry.addLine("=== DELTAS (SHOULD CHANGE!) ===");
            telemetry.addData("Delta X (goal-robot)", "%.2f cm", deltaX);
            telemetry.addData("Delta Y (goal-robot)", "%.2f cm", deltaY);
            telemetry.addData("Distance to Goal", "%.2f cm", distance);
            telemetry.addLine();

            telemetry.addLine("=== TURRET CALCULATION DEBUG ===");
            telemetry.addData("Auto-Aim Active", visionTrackingEnabled ? "YES (RT held)" : "YES (idle)");
            telemetry.addLine();
            telemetry.addData("Turret sees Robot X", "%.2f", turret.debugRobotX);
            telemetry.addData("Turret sees Robot Y", "%.2f", turret.debugRobotY);
            telemetry.addData("Turret sees Target X", "%.2f", turret.debugTargetX);
            telemetry.addData("Turret sees Target Y", "%.2f", turret.debugTargetY);
            telemetry.addLine();
            telemetry.addData("Turret Delta X", "%.2f cm", turret.debugDeltaX);
            telemetry.addData("Turret Delta Y", "%.2f cm", turret.debugDeltaY);
            telemetry.addData("Target Direction", "%.1f°", turret.debugTargetDirectionDeg);
            telemetry.addData("Robot Heading", "%.1f°", turret.debugRobotHeadingDeg);
            telemetry.addData("Calculated Angle", "%.1f°", turret.debugCalculatedAngleDeg);
            telemetry.addLine();

            // Проверка что deltas меняются
            if (Math.abs(turret.debugDeltaX) < 0.1 && Math.abs(turret.debugDeltaY) < 0.1) {
                telemetry.addLine("⚠️ WARNING: Deltas are near zero!");
                telemetry.addLine("   Robot might be ON the goal!");
            }
            if (Math.abs(deltaX - turret.debugDeltaX) > 0.5 || Math.abs(deltaY - turret.debugDeltaY) > 0.5) {
                telemetry.addLine("⚠️ WARNING: Deltas mismatch!");
                telemetry.addLine("   Check coordinate units!");
            }
        }

        // Vision (только в NORMAL режиме)
        if (currentMode == ControlMode.NORMAL) {
            telemetry.addLine("--- VISION ---");
            telemetry.addData("Tracking", visionTrackingEnabled ? "ACTIVE (RT held)" : "OFF");
            telemetry.addData("Alliance", vision.getAllianceColor());
            telemetry.addData("Target Tag ID", vision.getTargetTagId());
            telemetry.addData("Tag Visible", vision.hasTargetTag() ? "YES ✓" : "NO");

            if (vision.hasTargetTag()) {
                double distance = vision.getTargetDistance();
                double yaw = vision.getTargetYaw();
                telemetry.addData("Distance (cm)", "%.1f", distance);
                telemetry.addData("Yaw (turret angle)", "%.1f°", yaw);
            } else {
                telemetry.addData("Distance", "---");
                telemetry.addData("Yaw", "---");
            }
            telemetry.addLine();

            // Tracking mode indicator
            telemetry.addLine("--- AUTO-AIM MODE ---");
            if (turret.hasGoal()) {
                telemetry.addData("Active Mode", "ODOMETRY (Priority 1) ✓");
            } else if (vision.hasTargetTag()) {
                telemetry.addData("Active Mode", "VISION (Priority 2)");
            } else {
                telemetry.addData("Active Mode", "MANUAL");
            }
            telemetry.addLine();
        }

        // Controls reminder
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("Left Bumper", "Toggle Alliance (Current: " + (isRedAlliance ? "RED" : "BLUE") + ")");
        telemetry.addLine();
        if (currentMode == ControlMode.NORMAL) {
            telemetry.addData("Auto-Aim", "ALWAYS active (odometry)");
            telemetry.addData("RT (HOLD)", visionTrackingEnabled ? "Vision ON ✓" : "Vision available");
            telemetry.addData("Dpad Up", "CENTER (0°)");
            telemetry.addData("Right Bumper", "RED (%.0f°)", RED_POSITION);
            telemetry.addData("Dpad Down", "BLUE (%.0f°)", BLUE_POSITION);
            telemetry.addData("X", "Switch to OVERRIDE");
        } else {
            telemetry.addData("Dpad Right", "Move RIGHT");
            telemetry.addData("X", "Switch to NORMAL");
            telemetry.addLine("⚠️ OVERRIDE MODE ACTIVE");
        }
        telemetry.addData("A", "Stop");
        telemetry.addData("B", "Reset encoder");
    }

}
