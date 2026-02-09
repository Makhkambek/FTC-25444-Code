package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem {

    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;

    public DriveSubsystem(HardwareMap hw) {
        leftFront  = hw.get(DcMotorEx.class, "leftFront");
        rightFront = hw.get(DcMotorEx.class, "rightFront");
        leftRear   = hw.get(DcMotorEx.class, "leftRear");
        rightRear  = hw.get(DcMotorEx.class, "rightRear");

        // Verified directions: all positive power = forward

        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        // rightRear stays FORWARD (default)

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Mecanum drive.
     * @param y     Forward/backward  (-1 to 1). Positive = forward.
     * @param x     Strafe left/right (-1 to 1). Positive = right.
     * @param turn  Turn left/right   (-1 to 1). Positive = turn right.
     */
    public void driveMecanum(double y, double x, double turn) {
        double lf =  y + x + turn;
        double rf =  y - x - turn;
        double lr =  y - x + turn;
        double rr =  y + x - turn;

        // Normalize: scale down if any motor exceeds 1.0
        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lr), Math.abs(rr)))));

        leftFront.setPower(lf / max);
        rightFront.setPower(rf / max);
        leftRear.setPower(lr / max);
        rightRear.setPower(rr / max);
    }

    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public int getLeftFrontEncoder()  { return leftFront.getCurrentPosition(); }
    public int getRightFrontEncoder() { return rightFront.getCurrentPosition(); }
    public int getLeftRearEncoder()   { return leftRear.getCurrentPosition(); }
    public int getRightRearEncoder()  { return rightRear.getCurrentPosition(); }
}
