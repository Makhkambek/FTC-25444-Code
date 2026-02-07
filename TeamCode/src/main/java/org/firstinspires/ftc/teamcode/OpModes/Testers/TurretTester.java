package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;

@Config
@TeleOp(name="[PID] Turret PIDF Tuner", group="Testers")
public class TurretTester extends LinearOpMode {

    // PIDF коэффициенты (настраиваются через Dashboard)
    public static double KP = 0.018;
    public static double KI = 0.0;
    public static double KD = 0.0013;
    public static double KF = 0.002;

    // Preset positions для тестирования (в градусах)
    public static double CENTER_POSITION = 0.0;
    public static double LEFT_45 = -45.0;
    public static double RIGHT_45 = 45.0;
    public static double LEFT_90 = -90.0;
    public static double RIGHT_90 = 90.0;

    private Turret turret;

    // Button states
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;
    private boolean prevA = false;
    private boolean prevB = false;

    @Override
    public void runOpMode() {
        // FTC Dashboard телеметрия
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Инициализация турели (только hardwareMap, без Vision/Odometry)
        turret = new Turret(hardwareMap);

        telemetry.addLine("=== TURRET PIDF TUNER ===");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("Dpad Up: Center (0°)");
        telemetry.addLine("Dpad Left: Left 45° (-45°)");
        telemetry.addLine("Dpad Right: Right 45° (+45°)");
        telemetry.addLine("Left Bumper: Left 90° (-90°)");
        telemetry.addLine("Right Bumper: Right 90° (+90°)");
        telemetry.addLine("A: Stop turret");
        telemetry.addLine("B: Reset encoder");
        telemetry.addLine();
        telemetry.addLine("Tune PIDF via FTC Dashboard:");
        telemetry.addLine("192.168.43.1:8080");
        telemetry.addLine();
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        waitForStart();

        // Reset encoder at start
        turret.resetEncoder();

        while (opModeIsActive()) {
            // Update PIDF coefficients from Dashboard
            turret.setPIDF(KP, KI, KD, KF);

            handleControls();

            // CRITICAL: Apply PIDF to maintain target angle
            turret.maintainTarget();

            displayTelemetry();
            telemetry.update();
        }

        turret.stop();
    }

    private void handleControls() {
        // Dpad Up: Center (0°)
        if (gamepad1.dpad_up && !prevDpadUp) {
            turret.setTargetAngle(CENTER_POSITION);
        }

        // Dpad Left: Left 45° (-45°)
        if (gamepad1.dpad_left && !prevDpadLeft) {
            turret.setTargetAngle(LEFT_45);
        }

        // Dpad Right: Right 45° (+45°)
        if (gamepad1.dpad_right && !prevDpadRight) {
            turret.setTargetAngle(RIGHT_45);
        }

        // Left Bumper: Left 90° (-90°)
        if (gamepad1.left_bumper && !prevLeftBumper) {
            turret.setTargetAngle(LEFT_90);
        }

        // Right Bumper: Right 90° (+90°)
        if (gamepad1.right_bumper && !prevRightBumper) {
            turret.setTargetAngle(RIGHT_90);
        }

        // A: Stop
        if (gamepad1.a && !prevA) {
            turret.stop();
        }

        // B: Reset encoder
        if (gamepad1.b && !prevB) {
            turret.resetEncoder();
        }

        // Update button states
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevLeftBumper = gamepad1.left_bumper;
        prevRightBumper = gamepad1.right_bumper;
        prevA = gamepad1.a;
        prevB = gamepad1.b;
    }

    private void displayTelemetry() {
        double currentAngle = turret.getCurrentAngle();
        double targetAngle = turret.getTargetAngle();
        double error = targetAngle - currentAngle;
        double motorPower = turret.getMotorPower();

        telemetry.addLine("=== TURRET PIDF TUNER ===");
        telemetry.addLine();

        // Position
        telemetry.addLine("--- POSITION ---");
        telemetry.addData("Current Angle", "%.2f°", currentAngle);
        telemetry.addData("Target Angle", "%.2f°", targetAngle);
        telemetry.addData("Error", "%.2f°", error);
        telemetry.addData("At Target", turret.atTarget() ? "YES ✓" : "NO");
        telemetry.addLine();

        // Motor
        telemetry.addLine("--- MOTOR ---");
        telemetry.addData("Motor Power", "%.3f", motorPower);
        telemetry.addData("Current Ticks", "%.0f", turret.getCurrentPosition());
        telemetry.addData("Target Ticks", "%.0f", turret.getTargetPosition());
        telemetry.addLine();

        // PIDF Gains (from Dashboard)
        telemetry.addLine("--- PIDF GAINS (Dashboard) ---");
        telemetry.addData("kP", "%.5f", KP);
        telemetry.addData("kI", "%.5f", KI);
        telemetry.addData("kD", "%.5f", KD);
        telemetry.addData("kF", "%.5f", KF);
        telemetry.addLine();

        // Controls
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("Dpad Up", "Center (0°)");
        telemetry.addData("Dpad Left", "Left 45° (-45°)");
        telemetry.addData("Dpad Right", "Right 45° (+45°)");
        telemetry.addData("Left Bumper", "Left 90° (-90°)");
        telemetry.addData("Right Bumper", "Right 90° (+90°)");
        telemetry.addData("A", "Stop");
        telemetry.addData("B", "Reset encoder");
        telemetry.addLine();

        // Tuning Tips
        telemetry.addLine("--- TUNING TIPS ---");
        if (Math.abs(error) > 10.0) {
            telemetry.addLine("⚠️ Large error → Increase kP");
        } else if (Math.abs(error) > 5.0) {
            telemetry.addLine("✓ Getting close → Add kD");
        } else if (Math.abs(error) > 2.0) {
            telemetry.addLine("✓ Good! Fine tune kF");
        } else {
            telemetry.addLine("✅ Excellent!");
        }

        if (Math.abs(motorPower) < 0.01 && Math.abs(error) > 2.0) {
            telemetry.addLine("⚠️ Motor not moving → Increase kP or kF");
        }

        if (!turret.atTarget() && Math.abs(error) < 2.0) {
            telemetry.addLine("⚠️ Oscillating → Reduce kP or add kD");
        }
    }
}
