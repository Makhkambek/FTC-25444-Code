package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TailSubsystem {

    private final DcMotorEx tailMotor;

    // PID coefficients (tune only kP first)
    private double kP = 0.012;
    private double kI = 0.000008;
    private double kD = 0.00025;

    // Gravity feedforward (optional)
    private double kF = 0.0;   // set to 0.05–0.15 ONLY if your arm droops badly

    // Position limits (protect hardware)
    private int minPos = 0;
    private int maxPos = 1300;

    // State
    private int targetPosition = 0;
    private double integral = 0;
    private double lastPos = 0;

    private final ElapsedTime timer = new ElapsedTime();
    private double maxPower = 0.55;

    public TailSubsystem(HardwareMap hw) {

        tailMotor = hw.get(DcMotorEx.class, "armMotor");

        tailMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tailMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tailMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tailMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        timer.reset();
    }

    // Set a safe target
    public void setTargetPosition(int ticks) {
        targetPosition = ticks;
    }


    public int getCurrentPosition() { return tailMotor.getCurrentPosition(); }
    public int getTargetPosition() { return targetPosition; }

    public void stop() {
        tailMotor.setPower(0);
    }

    public void updatePID() {

        double dt = Math.max(timer.seconds(), 0.001);
        timer.reset();

        double pos = tailMotor.getCurrentPosition();
        double error = targetPosition - pos;

        // --- Integral control with decay (prevents windup) ---
        integral = (integral * 0.88) + error * dt;

        // --- Velocity-based derivative (very stable) ---
        double velocity = (pos - lastPos) / dt;
        lastPos = pos;

        // --- Combined PID + optional feedforward ---
        double output =
                (kP * error) +
                        (kI * integral) -
                        (kD * velocity) +
                        (kF);

        // Clamp to safe power
        output = clamp(output, -maxPower, maxPower);

        tailMotor.setPower(output);
    }

    public void manual(double power) {
        tailMotor.setPower(power);
        targetPosition = tailMotor.getCurrentPosition();
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
