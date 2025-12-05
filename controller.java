package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain  {
    private final DcMotor leftFront, leftBack, rightFront, rightBack;
    public DriveSubsystem(HardwareMap hw) {
        leftFront  = hw.dcMotor.get("leftFront");
        leftBack   = hw.dcMotor.get("leftBack");
        rightFront = hw.dcMotor.get("rightFront");
        rightBack  = hw.dcMotor.get("rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void drive (double drive, double turn, boolean slowmode){
    double leftPower = drive -turn;
    double rightPower = drive +turn;

    if (slowmode){
        leftPower *= 0.5;
        rightPower *= 0.5;
    }
    leftFront.setPower(leftPower);
    leftBack.setPower(leftPower);
    rightFront.setPower(rightPower);
    rightBack.setPower(rightPower);
    }
  //  @Override public void periodic() {}
}
