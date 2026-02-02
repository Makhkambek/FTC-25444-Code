package org.firstinspires.ftc.teamcode.Controller;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;

/**
 * IntakeController: Manages intake control based on gamepad1 input.
 * Right trigger → intake in
 * Right bumper → outtake
 */
public class IntakeController {

    private final IntakeSubsystem intake;

    // Trigger threshold
    private static final double TRIGGER_THRESHOLD = 0.1;

    /**
     * Constructs an IntakeController.
     * @param intake The IntakeSubsystem to control
     */
    public IntakeController(IntakeSubsystem intake) {
        this.intake = intake;
    }

    /**
     * Updates intake control based on gamepad1 input.
     * Call this every loop.
     * @param gamepad1 The gamepad to read from
     */
    public void update(Gamepad gamepad1) {
        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            // Right trigger pressed → intake in
            intake.intakeIn();
        } else if (gamepad1.right_bumper) {
            // Right bumper pressed → outtake
            intake.intakeOut();
        } else {
            // No input → stop
            intake.stop();
        }
    }
}