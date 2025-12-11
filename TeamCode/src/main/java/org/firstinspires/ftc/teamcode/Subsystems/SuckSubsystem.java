package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SuckSubsystem {

    private final DcMotor suck;

    public SuckSubsystem(HardwareMap hw) {
        suck = hw.get(DcMotor.class, "suck");

        suck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intake() {
        suck.setPower(1.0);   // forward
    }

    public void outtake() {
        suck.setPower(-1.0);  // reverse
    }

    public void stop() {
        suck.setPower(0.0);
    }
}