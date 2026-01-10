package org.firstinspires.ftc.teamcode.Controller;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;

public class IntakeController {

    private final IntakeSubsystem intake;
    private static final double DEADZONE = 0.1;

    public IntakeController(IntakeSubsystem intake) {
        this.intake = intake;
    }

    public void update(Gamepad gamepad) {

        if (gamepad.right_trigger > DEADZONE) {
            intake.intakeIn();
        }
        else if (gamepad.left_trigger > DEADZONE) {
            intake.intakeOut();
        }
        else {
            intake.stop();
        }
    }
}
