package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

@Config
@TeleOp(name="[TEST] Turret Tester", group="Testers")
public class TurretTester extends LinearOpMode {

    // PIDF коэффициенты для настройки через Dashboard
    public static double KP = 0.02;
    public static double KI = 0.0;
    public static double KD = 0.009;
    public static double KF = 0.0007; // Feedforward для преодоления трения

    // Test positions (в градусах)
    public static double RED_POSITION = 45.0;   // Красная корзина
    public static double BLUE_POSITION = -45.0; // Синяя корзина

    private Turret turret;
    private Vision vision;

    private boolean prevDpadUp = false;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false; // Для OVERRIDE MODE toggle
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;

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

        // Инициализация Vision
        vision = new Vision();
        vision.init(hardwareMap);
        vision.setAlliance(true); // По умолчанию RED (можно поменять на false для BLUE)

        // Создаем Turret с Vision (БЕЗ Localizer - он не нужен для Vision tracking)
        turret = new Turret(hardwareMap, vision);

        telemetry.addLine("=== TURRET TESTER ===");
        telemetry.addLine();
        telemetry.addLine("NORMAL MODE:");
        telemetry.addLine("Right Stick X: Manual control (PIDF)");
        telemetry.addLine("Right Trigger (HOLD): Vision auto-aim");
        telemetry.addLine("Dpad Up: CENTER (0°)");
        telemetry.addLine("Right Bumper: RED position (45°)");
        telemetry.addLine("Left Bumper: BLUE position (-45°)");
        telemetry.addLine();
        telemetry.addLine("OVERRIDE MODE (X to toggle):");
        telemetry.addLine("Dpad Left: Move LEFT (direct power)");
        telemetry.addLine("Dpad Right: Move RIGHT (direct power)");
        telemetry.addLine();
        telemetry.addLine("COMMON:");
        telemetry.addLine("A: Stop turret");
        telemetry.addLine("B: Reset encoder");
        telemetry.addLine();
        telemetry.addLine("Tune PIDF via FTC Dashboard");
        telemetry.update();

        waitForStart();

        // Запускаем Vision
        vision.start();

        // Reset encoder
        turret.resetEncoder();

        while (opModeIsActive()) {
            // Обновляем PIDF коэффициенты из Dashboard
            turret.setPIDF(KP, KI, KD, KF);

            handleControls();
            displayTelemetry();
            telemetry.update();
        }

        turret.stop();
        vision.stop();
    }

    private void handleControls() {
        // Переключение режимов (X)
        if (gamepad1.x && !prevX) {
            currentMode = (currentMode == ControlMode.NORMAL) ? ControlMode.OVERRIDE : ControlMode.NORMAL;
            turret.syncManualTarget(); // Синхронизируем при переключении
        }

        if (currentMode == ControlMode.NORMAL) {
            // === NORMAL MODE - PIDF Control ===

            // Vision tracking (HOLD Right Trigger)
            if (gamepad1.right_trigger > 0.5) {
                visionTrackingEnabled = true;
                turret.autoAim(); // Автоприцеливание через Vision
            } else {
                visionTrackingEnabled = false;

                // Manual control с джойстиком
                double stickInput = -gamepad1.right_stick_x;
                if (Math.abs(stickInput) > 0.1) {
                    turret.manualControl(stickInput);
                } else {
                    // Держим текущий targetAngle
                    turret.autoAim();
                }

                // Preset positions
                if (gamepad1.dpad_up && !prevDpadUp) {
                    turret.returnToCenter(); // CENTER (0°)
                }

                if (gamepad1.right_bumper && !prevRightBumper) {
                    turret.setTargetAngle(RED_POSITION); // RED (45°)
                }

                if (gamepad1.left_bumper && !prevLeftBumper) {
                    turret.setTargetAngle(BLUE_POSITION); // BLUE (-45°)
                }
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

        // PIDF коэффициенты (только в NORMAL режиме)
        if (currentMode == ControlMode.NORMAL) {
            telemetry.addLine("--- PIDF (Dashboard) ---");
            telemetry.addData("kP", "%.4f", KP);
            telemetry.addData("kI", "%.4f", KI);
            telemetry.addData("kD", "%.4f", KD);
            telemetry.addData("kF", "%.4f", KF);
            telemetry.addLine();
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
        }

        // Controls reminder
        telemetry.addLine("--- CONTROLS ---");
        if (currentMode == ControlMode.NORMAL) {
            telemetry.addData("RT (HOLD)", visionTrackingEnabled ? "Vision ON ✓" : "Vision auto-aim");
            telemetry.addData("Right Stick X", "Manual (PIDF)");
            telemetry.addData("Dpad Up", "CENTER (0°)");
            telemetry.addData("Right Bumper", "RED (%.0f°)", RED_POSITION);
            telemetry.addData("Left Bumper", "BLUE (%.0f°)", BLUE_POSITION);
            telemetry.addData("X", "Switch to OVERRIDE");
        } else {
            telemetry.addData("Dpad Left", "Move LEFT");
            telemetry.addData("Dpad Right", "Move RIGHT");
            telemetry.addData("X", "Switch to NORMAL");
            telemetry.addLine("⚠️ OVERRIDE MODE ACTIVE");
        }
        telemetry.addData("A", "Stop");
        telemetry.addData("B", "Reset encoder");
    }

}
