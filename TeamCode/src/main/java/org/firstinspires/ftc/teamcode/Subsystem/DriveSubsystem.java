package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveSubsystem {

    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;

    private double speedMultiplier = 1.0;

    public DriveSubsystem(HardwareMap hw) {
        leftFront  = hw.get(DcMotorEx.class, "leftFront");
        rightFront = hw.get(DcMotorEx.class, "rightFront");
        leftRear   = hw.get(DcMotorEx.class, "leftRear");
        rightRear  = hw.get(DcMotorEx.class, "rightRear");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);

        setBrake(true);
        setRunWithoutEncoders();
        resetEncoders();
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(Gamepad gamepad1) {
        double slowModeFactor = gamepad1.right_trigger > 0.1 ? 0.3 : 1.0;

        double y = -gamepad1.left_stick_y * slowModeFactor;
        double x = gamepad1.left_stick_x * slowModeFactor;
        double turn = -gamepad1.right_stick_x * slowModeFactor;

        y = shapeInput(y);
        x = shapeInput(x);
        turn = shapeInput(turn);

        double lf = y + x + turn;
        double rf = y - x - turn;
        double lr = y - x + turn;
        double rr = y + x - turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lr), Math.abs(rr)))));

        leftFront.setPower((lf / max) * speedMultiplier);
        rightFront.setPower((rf / max) * speedMultiplier);
        leftRear.setPower((lr / max) * speedMultiplier);
        rightRear.setPower((rr / max) * speedMultiplier);
    }

    public int getLeftFrontEncoder() {
        return leftFront.getCurrentPosition();
    }

    public int getRightFrontEncoder() {
        return rightFront.getCurrentPosition();
    }

    public int getLeftRearEncoder() {
        return leftRear.getCurrentPosition();
    }

    public int getRightRearEncoder() {
        return rightRear.getCurrentPosition();
    }

    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = clamp(multiplier, 0.0, 1.0);
    }

    private void setRunWithoutEncoders() {
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setBrake(boolean brake) {
        DcMotorEx.ZeroPowerBehavior behavior =
                brake ? DcMotorEx.ZeroPowerBehavior.BRAKE
                        : DcMotorEx.ZeroPowerBehavior.FLOAT;
        leftFront.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        leftRear.setZeroPowerBehavior(behavior);
        rightRear.setZeroPowerBehavior(behavior);
    }

    private double shapeInput(double value) {
        return value * value * value;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}