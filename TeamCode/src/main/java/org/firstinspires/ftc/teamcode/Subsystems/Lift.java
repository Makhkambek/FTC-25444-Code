package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift {
    public enum State { GND, HANG, MID, HIGH, IDLE }

    private final DcMotor leftLift;
    private final DcMotor rightLift;
    private final DcMotor middleLift;

    private State currentState = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    private double kP = 0.0035;
    private double kI = 0.0;
    private double kD = 0.0;
    private double integral = 0;
    private double lastError = 0;

    public static final int POS_GND = 0;
    public static final int POS_HANG = 350;
    public static final int POS_MID = 800;
    public static final int POS_HIGH = 1500;

    private int targetTicks = 0;

    public Lift(HardwareMap hw) {
        leftLift = hw.get(DcMotor.class, "leftLift");
        rightLift = hw.get(DcMotor.class, "rightLift");
        middleLift = hw.get(DcMotor.class, "middleLift");

        rightLift.setDirection(DcMotor.Direction.REVERSE);
        middleLift.setDirection(DcMotor.Direction.REVERSE);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middleLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timer.reset();
    }

    public void update() {
        if (currentState == State.IDLE) {
            leftLift.setPower(0);
            rightLift.setPower(0);
            middleLift.setPower(0);
            integral = 0;
            lastError = 0;
            timer.reset();
            return;
        }

        switch (currentState) {
            case GND: targetTicks = POS_GND; break;
            case HANG: targetTicks = POS_HANG; break;
            case MID: targetTicks = POS_MID; break;
            case HIGH: targetTicks = POS_HIGH; break;
            default: targetTicks = getCurrentPosition();
        }

        int current = getCurrentPosition();
        double dt = Math.max(0.001, timer.seconds());
        timer.reset();

        double error = targetTicks - current;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        double power = (kP * error) + (kI * integral) + (kD * derivative);

        if (power > 1.0) power = 1.0;
        if (power < -1.0) power = -1.0;

        if (Math.abs(error) < 10) power = 0;

        leftLift.setPower(power);
        rightLift.setPower(power);
        middleLift.setPower(power);

        lastError = error;
    }

    public void setState(State s) {
        currentState = s;
    }

    public void setTargetTicks(int ticks) {
        targetTicks = ticks;
        currentState = State.IDLE;
    }

    public int getCurrentPosition() {
        return leftLift.getCurrentPosition();
    }
}
