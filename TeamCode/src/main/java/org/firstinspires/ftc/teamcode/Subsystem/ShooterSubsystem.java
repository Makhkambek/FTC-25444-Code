package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {

    private final DcMotorEx shooterMotor;

    // Shooter speeds
    private static final double DEFAULT_POWER = 0.5;
    private static final double MAX_POWER = 1.0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setPower(DEFAULT_POWER); // always run at 50% initially
    }

    public void setDefaultSpeed() {
        shooterMotor.setPower(DEFAULT_POWER);
    }

    public void setMaxSpeed() {
        shooterMotor.setPower(MAX_POWER);
    }

    public void stop() {
        shooterMotor.setPower(0);
    }
}
