package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    private final DcMotorEx intakeMotor;

    // Adjust power if needed
    private static final double INTAKE_POWER = 1.0;
    private static final double OUTTAKE_POWER = -1.0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void intakeIn() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void intakeOut() {
        intakeMotor.setPower(OUTTAKE_POWER);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}
