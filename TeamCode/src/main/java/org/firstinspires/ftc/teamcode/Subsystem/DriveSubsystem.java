package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem {

    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;

    // Speed scaling (can be changed by controller)
    private double speedMultiplier = 1.0;

    public DriveSubsystem(HardwareMap hw) {
        leftFront  = hw.get(DcMotorEx.class, "leftFront");
        rightFront = hw.get(DcMotorEx.class, "rightFront");
        leftRear   = hw.get(DcMotorEx.class, "leftRear");
        rightRear  = hw.get(DcMotorEx.class, "rightRear");

        // Reverse left side
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);

        // Brake behavior
        setBrake(true);

        // Explicit motor mode (IMPORTANT)
        setRunWithoutEncoders();
    }

    /* ================= CORE DRIVE ================= */

    public void drive(double y, double x, double turn) {

        // Input shaping (cubic = smoother low-speed control)
        y = shapeInput(y);
        x = shapeInput(x);
        turn = shapeInput(turn);

        double lf = y + x + turn;
        double rf = y - x - turn;
        double lr = y - x + turn;
        double rr = y + x - turn;

        // Normalize
        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lr), Math.abs(rr)))));

        leftFront.setPower((lf / max) * speedMultiplier);
        rightFront.setPower((rf / max) * speedMultiplier);
        leftRear.setPower((lr / max) * speedMultiplier);
        rightRear.setPower((rr / max) * speedMultiplier);
    }

    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    /* ================= SPEED CONTROL ================= */

    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = clamp(multiplier, 0.0, 1.0);
    }

    /* ================= CONFIG HELPERS ================= */

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

    /* ================= UTILS ================= */

    private double shapeInput(double value) {
        return value * value * value; // cubic shaping
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
