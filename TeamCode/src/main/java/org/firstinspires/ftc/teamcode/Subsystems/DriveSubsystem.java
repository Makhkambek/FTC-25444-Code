package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem  {

    private final DcMotor leftFront, leftBack, rightFront, rightBack;

    public DriveSubsystem(HardwareMap hw) {
        leftFront  = hw.dcMotor.get("leftFront");
        leftBack   = hw.dcMotor.get("leftBack");
        rightFront = hw.dcMotor.get("rightFront");
        rightBack  = hw.dcMotor.get("rightBack");

        // These reversals are correct for standard mecanum
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private double dz(double v) { return Math.abs(v) < 0.07 ? 0 : v; }
    private double smooth(double v) { return v * v * v; }

    // holonomic mecanum drive
    public void drive(double y, double x, double rx, boolean slow) {

        // Deadzone + cubic smoothing
        y  = smooth(dz(y));
        x  = smooth(dz(x));
        rx = smooth(dz(rx));

        double mult = slow ? 0.35 : 1.0;

        double lf = (y + x + rx) * mult;
        double lb = (y - x + rx) * mult;
        double rf = (y - x - rx) * mult;
        double rb = (y + x - rx) * mult;

        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }
}
