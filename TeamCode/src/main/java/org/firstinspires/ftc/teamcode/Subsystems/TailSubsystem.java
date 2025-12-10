package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TailSubsystem {

    private final DcMotorEx tailMotor;

    private static final double kP = 0.015;
    private static final double kI = 0.0006;
    private static final double kD = 0.001;
    private static final double kF = 0.08;     // feedforward to fight gravity

    private static final double HOLD_POWER = 0.05;

    private double integralSum = 0;
    private double lastError = 0;

    private final ElapsedTime timer = new ElapsedTime();

    // Anti-windup and stability thresholds
    private static final double INTEGRAL_LIMIT = 200;
    private static final int STABLE_THRESHOLD = 10;

    // Position setpoint
    private int targetPosition = 0;

    public TailSubsystem(HardwareMap hw) {

        tailMotor = hw.get(DcMotorEx.class, "armMotor");
        tailMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tailMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tailMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        timer.reset();
    }

    public void setTargetPosition(int ticks) {
        targetPosition = ticks;
    }
    public int getCurrentPosition() {
        return tailMotor.getCurrentPosition();
    }
    public int getTargetPosition() {
        return targetPosition;
    }
    public void stop() {
        tailMotor.setPower(0);
    }

    public void updatePID() {

        double dt = Math.max(timer.seconds(), 0.001);
        timer.reset();

        double current = tailMotor.getCurrentPosition();
        double error = targetPosition - current;

        // Integral
        if (Math.abs(error) < STABLE_THRESHOLD) {
            integralSum = 0;
        } else {
            integralSum += error * dt;
            integralSum = clamp(integralSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        }

        double derivative = (error - lastError) / dt;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);
        output += Math.signum(error) * kF;

        if (Math.abs(error) < STABLE_THRESHOLD) {
            output = HOLD_POWER * Math.signum(error);
        }

        tailMotor.setPower(clamp(output, -1.0, 1.0));
        lastError = error;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
