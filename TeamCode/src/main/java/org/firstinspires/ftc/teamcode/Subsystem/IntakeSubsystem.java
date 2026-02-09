package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * IntakeSubsystem: Controls the intake/transfer motor.
 * Simple power-based control for intake and outtake operations.
 */
public class IntakeSubsystem {

    // Hardware
    private final DcMotorEx intakeMotor;

    // Power constants
    private static final double IN_POWER = -1.0;
    private static final double OUT_POWER = 1.0;

    /**
     * Constructs an IntakeSubsystem.
     * @param hardwareMap Hardware map from the opmode
     */
    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        // Configuration
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Runs intake inward at full power.
     */
    public void intakeIn() {
        intakeMotor.setPower(IN_POWER);
    }

    /**
     * Runs intake outward (outtake) at full power.
     */
    public void intakeOut() {
        intakeMotor.setPower(OUT_POWER);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        intakeMotor.setPower(0.0);
    }
}