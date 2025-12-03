package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TailSubsystem {

    private final DcMotorEx tailMotor;

    // Power values (can adjust after testing)
    private final double TAIL_UP_POWER   = 1.0;
    private final double TAIL_DOWN_POWER = -1.0;
    private final double TAIL_STOP_POWER = 0.0;

    public TailSubsystem(HardwareMap hw) {
        tailMotor = hw.get(DcMotorEx.class, "armMotor");

        // REQUIRED so setPower() actually works
        tailMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Helps with holding position
        tailMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Try reverse — remove this if motion goes opposite direction
        tailMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void up() {
        tailMotor.setPower(TAIL_UP_POWER);
    }

    public void down() {
        tailMotor.setPower(TAIL_DOWN_POWER);
    }

    public void stop() {
        tailMotor.setPower(TAIL_STOP_POWER);
    }
}