package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;
// import org.firstinspires.ftc.teamcode.SubSystems.Vision;

@Config
@TeleOp(name="[TEST] Turret Tester", group="Testers")
public class TurretTester extends LinearOpMode {

    // PID коэффициенты для настройки через Dashboard
    public static double KP = 0.03;
    public static double KI = 0.0;
    public static double KD = 0.01;

    // Test positions
    public static double RED_POSITION = 100;
    public static double BLUE_POSITION = -100;

    private Turret turret;

    private boolean prevDpadUp = false;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;

    // Режим управления
    private enum ControlMode {
        NORMAL,    // Обычный режим с PID
        OVERRIDE   // Аварийный режим - прямое управление мощностью
    }
    private ControlMode currentMode = ControlMode.NORMAL;

    @Override
    public void runOpMode() {
        // Настройка телеметрии для Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new Turret(hardwareMap);

        telemetry.addLine("=== TURRET TESTER ===");
        telemetry.addLine();
        telemetry.addLine("NORMAL MODE:");
        telemetry.addLine("Right Stick X: Manual control (PID)");
        telemetry.addLine("Dpad Up: CENTER (0)");
        telemetry.addLine();
        telemetry.addLine("OVERRIDE MODE (X to toggle):");
        telemetry.addLine("Dpad Left: Move LEFT (direct power)");
        telemetry.addLine("Dpad Right: Move RIGHT (direct power)");
        telemetry.addLine();
        telemetry.addLine("COMMON:");
        telemetry.addLine("A: Stop turret");
        telemetry.addLine("B: Reset encoder");
        telemetry.addLine();
        telemetry.addLine("Tune PID via FTC Dashboard");
        telemetry.update();

        waitForStart();

        // Reset encoder
        turret.resetEncoder();

        while (opModeIsActive()) {
            // Обновляем PID коэффициенты из Dashboard
            turret.setPID(KP, KI, KD);

            handleControls();
            displayTelemetry();
            telemetry.update();
        }

        turret.stop();
    }

    private void handleControls() {
        // Переключение режимов
        if (gamepad1.x && !prevX) {
            currentMode = (currentMode == ControlMode.NORMAL) ? ControlMode.OVERRIDE : ControlMode.NORMAL;
            turret.syncManualTarget(); // Синхронизируем при переключении
        }

        if (currentMode == ControlMode.NORMAL) {
            // === NORMAL MODE - PID Control ===

            // Manual control с джойстиком
            double stickInput = -gamepad1.right_stick_x;
            if (Math.abs(stickInput) > 0.1) {
                turret.manualControl(stickInput);
            } else {
                // Если джойстик не двигается - автопозиционирование к target
                turret.autoAim();
            }

            // Center preset
            if (gamepad1.dpad_up && !prevDpadUp) {
                turret.returnToCenter();
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
    }

    private void displayTelemetry() {
        telemetry.addLine("=== TURRET TESTER ===");
        telemetry.addLine();

        // Mode
        telemetry.addLine("--- MODE ---");
        String modeStr = (currentMode == ControlMode.NORMAL) ? "NORMAL (PID)" : "OVERRIDE (Direct)";
        telemetry.addData("Current Mode", modeStr);
        telemetry.addData("Switch Mode", "Press X");
        telemetry.addLine();

        // Position
        telemetry.addLine("--- POSITION ---");
        telemetry.addData("Current", "%.0f ticks", turret.getCurrentPosition());
        telemetry.addData("Target", "%.0f ticks", turret.getTargetPosition());
        telemetry.addData("Error", "%.0f ticks",
            turret.getTargetPosition() - turret.getCurrentPosition());
        telemetry.addData("At Target", turret.atTarget() ? "YES" : "NO");
        telemetry.addLine();

        // PID коэффициенты (только в NORMAL режиме)
        if (currentMode == ControlMode.NORMAL) {
            telemetry.addLine("--- PID (Dashboard) ---");
            telemetry.addData("kP", "%.3f", KP);
            telemetry.addData("kI", "%.3f", KI);
            telemetry.addData("kD", "%.3f", KD);
            telemetry.addLine();
        }

        // Controls reminder
        telemetry.addLine("--- CONTROLS ---");
        if (currentMode == ControlMode.NORMAL) {
            telemetry.addData("Right Stick X", "Manual (PID)");
            telemetry.addData("Dpad Up", "CENTER");
        } else {
            telemetry.addData("Dpad Left", "Move LEFT");
            telemetry.addData("Dpad Right", "Move RIGHT");
            telemetry.addLine("⚠️ OVERRIDE MODE ACTIVE");
        }
        telemetry.addData("A", "Stop");
        telemetry.addData("B", "Reset encoder");
    }

}
