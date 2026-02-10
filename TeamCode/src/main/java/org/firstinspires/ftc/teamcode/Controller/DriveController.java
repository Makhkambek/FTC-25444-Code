package org.firstinspires.ftc.teamcode.Controller;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;

public class DriveController {

    private final DriveSubsystem drive;

    private static final double DEADZONE = 0.1;
    private static final double SLOW_MODE_FACTOR = 0.4;

    public DriveController(DriveSubsystem drive) {
        this.drive = drive;
    }

    public void update(Gamepad gamepad1) {
        double y    = -gamepad1.left_stick_y;   // Positive = forward
        double x    =  gamepad1.left_stick_x;   // Positive = strafe right
        double turn =  gamepad1.right_stick_x;  // Positive = turn right

        // Apply deadzone
        y    = applyDeadzone(y);
        x    = applyDeadzone(x);
        turn = applyDeadzone(turn);

        // Slow mode with left trigger
        if (gamepad1.left_trigger > 0.1) {
            y    *= SLOW_MODE_FACTOR;
            x    *= SLOW_MODE_FACTOR;
            turn *= SLOW_MODE_FACTOR;
        }

        drive.driveMecanum(y, x, turn);
    }

    private double applyDeadzone(double value) {
        if (Math.abs(value) < DEADZONE) return 0.0;
        // Remap so output starts at 0 right after deadzone
        return (value > 0)
                ? (value - DEADZONE) / (1.0 - DEADZONE)
                : (value + DEADZONE) / (1.0 - DEADZONE);
    }
}
