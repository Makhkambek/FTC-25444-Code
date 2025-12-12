package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    public Drive(HardwareMap hw) {
        leftFront = hw.get(DcMotorEx.class, "leftFront");
        leftRear = hw.get(DcMotorEx.class, "leftRear");
        rightFront = hw.get(DcMotorEx.class, "rightFront");
        rightRear = hw.get(DcMotorEx.class, "rightRear");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);

    }

    public void drive(double x, double y, double rx) {
        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));
        if (max > 1) {
            fl /= max; bl /= max; fr /= max; br /= max;
        }

        leftFront.setPower(fl);
        leftRear.setPower(bl);
        rightFront.setPower(fr);
        rightRear.setPower(br);
    }
}
