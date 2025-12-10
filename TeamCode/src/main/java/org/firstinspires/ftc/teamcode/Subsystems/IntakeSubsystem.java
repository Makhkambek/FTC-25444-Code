package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    private final CRServo intakeLeft;
    private final CRServo intakeRight;

    private final double INTAKE_POWER = 1.0;
    private final double STOP_POWER   = 0.0;

    public IntakeSubsystem(HardwareMap hw) {
        intakeLeft  = hw.get(CRServo.class, "intakeLeft");
        intakeRight = hw.get(CRServo.class, "intakeRight");

        // If one spins backwards:
        intakeRight.setDirection(CRServo.Direction.REVERSE);
    }

    public void run() {
        intakeLeft.setPower(INTAKE_POWER);
        intakeRight.setPower(INTAKE_POWER);
    }
    public void stop() {
        intakeLeft.setPower(STOP_POWER);
        intakeRight.setPower(STOP_POWER);
    }
}
