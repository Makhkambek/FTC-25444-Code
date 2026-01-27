package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="[PID] Shooter Motors Only", group="Testers")
public class ShooterPIDTuner extends LinearOpMode {

    // PID коэффициенты (настраиваются через Dashboard)
    public static double KP = 0.011; //checked
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KF = 0.00041;  //checked

    // Настройки
    public static double TARGET_VELOCITY = 1700; // ticks/sec
    public static double INTEGRAL_LIMIT = 100.0; // Anti-windup

    // PID защита от спайков
    private static final double MIN_DELTA_TIME = 0.010; // минимум 10 мс
    private static final double MAX_DERIVATIVE = 500.0; // максимальное значение derivative

    // Защита от толчков
    private static final double OUTPUT_DEADBAND = 0.005; // минимальное изменение output для применения
    private static final double MAX_OUTPUT_CHANGE = 0.05; // максимальное изменение за один цикл (rate limiter)

    // Моторы
    private DcMotorEx shooterMotor1, shooterMotor2;

    // PID переменные
    private double lastError = 0;
    private double integralSum = 0;
    private double targetVelocity = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    // Сглаживание output для уменьшения дергания motor2
    private double smoothedOutput = 0;
    private static final double SMOOTHING_FACTOR = 0.9; // 0.0 = нет сглаживания, 1.0 = максимальное

    // Button states
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    @Override
    public void runOpMode() {
        // FTC Dashboard телеметрия
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Инициализация моторов
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");

        // Motor1 БЕЗ REVERSE - используется для чтения velocity в PID
        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Motor2 в REVERSE - для синхронного вращения
        shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        pidTimer.reset();

        telemetry.addLine("=== SHOOTER PID TUNER ===");
        telemetry.addLine("Motors Only - No Servos");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("A: Start Motors");
        telemetry.addLine("B: Stop Motors");
        telemetry.addLine("Dpad Up: +100 velocity");
        telemetry.addLine("Dpad Down: -100 velocity");
        telemetry.addLine();
        telemetry.addLine("Tune PID via FTC Dashboard");
        telemetry.addData("Status", "Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleControls();
            updatePID();
            displayTelemetry();
            telemetry.update();
        }

        // Cleanup
        stopMotors();
    }

    private void handleControls() {
        // A: Start motors
        if (gamepad1.a && !prevA) {
            targetVelocity = TARGET_VELOCITY;
            lastError = 0;
            integralSum = 0;
            smoothedOutput = 0;
            pidTimer.reset();
        }

        // B: Stop motors
        if (gamepad1.b && !prevB) {
            stopMotors();
        }

        // Dpad Up: Increase velocity
        if (gamepad1.dpad_up && !prevDpadUp) {
            TARGET_VELOCITY += 100;
        }

        // Dpad Down: Decrease velocity
        if (gamepad1.dpad_down && !prevDpadDown) {
            TARGET_VELOCITY -= 100;
            if (TARGET_VELOCITY < 0) TARGET_VELOCITY = 0;
        }

        // Update button states
        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
    }

    private void updatePID() {
        if (targetVelocity == 0) {
            shooterMotor1.setPower(0);
            shooterMotor2.setPower(0);
            return;
        }

        double currentVelocity = shooterMotor1.getVelocity();
        double error = targetVelocity - currentVelocity;

        double deltaTime = pidTimer.seconds();
        pidTimer.reset();

        // Защита от слишком маленького deltaTime (предотвращает derivative spike)
        if (deltaTime < MIN_DELTA_TIME) {
            return; // Пропускаем этот цикл, слишком быстро
        }

        // Вычисляем derivative
        double derivative = (error - lastError) / deltaTime;

        // Ограничиваем derivative для предотвращения спайков
        derivative = Math.max(-MAX_DERIVATIVE, Math.min(MAX_DERIVATIVE, derivative));

        // Обновляем integral
        integralSum += error * deltaTime;

        // Anti-windup
        integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));

        // PID calculation + Feedforward
        double output = (KP * error) + (KI * integralSum) + (KD * derivative) + (KF * targetVelocity);

        // Clamp output
        output = Math.max(-1.0, Math.min(1.0, output));

        // Сглаживание output через EMA (Exponential Moving Average)
        double newSmoothedOutput = (SMOOTHING_FACTOR * smoothedOutput) + ((1.0 - SMOOTHING_FACTOR) * output);

        // Rate limiter - ограничиваем скорость изменения мощности
        double outputChange = newSmoothedOutput - smoothedOutput;
        if (Math.abs(outputChange) > MAX_OUTPUT_CHANGE) {
            outputChange = Math.signum(outputChange) * MAX_OUTPUT_CHANGE;
            newSmoothedOutput = smoothedOutput + outputChange;
        }

        // Deadband - применяем изменение только если оно достаточно большое
        if (Math.abs(outputChange) > OUTPUT_DEADBAND) {
            smoothedOutput = newSmoothedOutput;
        }

        // Set power to both motors (используем сглаженный output)
        shooterMotor1.setPower(smoothedOutput);
        shooterMotor2.setPower(smoothedOutput);

        lastError = error;
    }

    private void stopMotors() {
        targetVelocity = 0;
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
        lastError = 0;
        integralSum = 0;
        smoothedOutput = 0;
    }

    private void displayTelemetry() {
        double currentVel = shooterMotor1.getVelocity();
        double error = targetVelocity - currentVel;

        telemetry.addLine("=== SHOOTER PID TUNER ===");
        telemetry.addData("Motors", "shooterMotor1, shooterMotor2");
        telemetry.addLine();

        // Status
        telemetry.addLine("--- STATUS ---");
        telemetry.addData("Running", targetVelocity > 0 ? "YES" : "NO");
        telemetry.addLine();

        // Velocity
        telemetry.addLine("--- VELOCITY ---");
        telemetry.addData("Target", "%.0f ticks/sec", targetVelocity);
        telemetry.addData("Current", "%.0f ticks/sec", currentVel);
        telemetry.addData("Error", "%.0f ticks/sec", error);

        if (targetVelocity > 0) {
            double accuracy = (currentVel / targetVelocity) * 100.0;
            telemetry.addData("Accuracy", "%.1f%%", accuracy);
        }
        telemetry.addLine();

        // PID Gains
        telemetry.addLine("--- PID GAINS (Dashboard) ---");
        telemetry.addData("kP", "%.5f", KP);
        telemetry.addData("kI", "%.5f", KI);
        telemetry.addData("kD", "%.5f", KD);
        telemetry.addData("kF", "%.5f", KF);
        telemetry.addData("Integral Sum", "%.2f", integralSum);
        telemetry.addLine();

        // Motor Power
        telemetry.addLine("--- MOTOR POWER ---");
        telemetry.addData("Smoothed Output", "%.3f", smoothedOutput);
        telemetry.addData("Motor1 Power", "%.3f", shooterMotor1.getPower());
        telemetry.addData("Motor2 Power", "%.3f", shooterMotor2.getPower());
        telemetry.addLine();

        // Controls
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("A", "Start motors");
        telemetry.addData("B", "Stop motors");
        telemetry.addData("Dpad Up", "+100 velocity");
        telemetry.addData("Dpad Down", "-100 velocity");
        telemetry.addLine();

        // Tuning Tips
        telemetry.addLine("--- TUNING TIPS ---");
        if (targetVelocity > 0) {
            if (Math.abs(error) > 200) {
                telemetry.addLine("⚠️ Large error → Increase kP");
            } else if (Math.abs(error) > 50) {
                telemetry.addLine("✓ Getting close → Add kD");
            } else if (Math.abs(error) > 20) {
                telemetry.addLine("✓ Good! Try small kI");
            } else {
                telemetry.addLine("✅ Excellent!");
            }

            if (currentVel > targetVelocity * 1.1) {
                telemetry.addLine("⚠️ Overshoot → Reduce kP or add kD");
            }
        } else {
            telemetry.addLine("Press A to start");
        }
    }
}
