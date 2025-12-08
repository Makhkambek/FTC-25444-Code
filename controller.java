package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.*;

public class Drivetrain {

    private DcMotor leftFront, leftBack, rightFront, rightBack;

    public Drivetrain(HardwareMap hw) {
        leftFront  = hw.get(DcMotor.class,"leftFront");
        leftBack   = hw.get(DcMotor.class,"leftBack");
        rightFront = hw.get(DcMotor.class,"rightFront");
        rightBack  = hw.get(DcMotor.class,"rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double drive, double turn, boolean slowmode) {

        double leftPower = drive - turn;
        double rightPower = drive + turn;

        if (slowmode) {
            leftPower *= 0.5;
            rightPower *= 0.5;
        }

        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);
    }
}
