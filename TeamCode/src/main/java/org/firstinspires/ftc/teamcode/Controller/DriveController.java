package org.firstinspires.ftc.teamcode.Controller;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;

/**
 * DriveController: Manages mecanum drive control based on gamepad1 input.
 * Left stick Y → forward/backward
 * Left stick X → strafe left/right
 * Right stick X → rotate
 * Left trigger → slow mode
 */
public class DriveController {

    private final DriveSubsystem drive;

    // Control constants
    private static final double DEADZONE = 0.1;
    private static final double SLOW_MODE_FACTOR = 0.3;
    private static final double SLOW_MODE_TRIGGER_THRESHOLD = 0.1;

    /**
     * Constructs a DriveController.
     * @param drive The DriveSubsystem to control
     */
    public DriveController(DriveSubsystem drive) {
        this.drive = drive;
    }

    /**
     * Updates drive control based on gamepad1 input.
     * Call this every loop.
     * @param gamepad1 The gamepad to read from
     */
    public void update(Gamepad gamepad1) {
        // Read joystick values
        double y = -gamepad1.left_stick_y;  // Inverted for FTC coordinate system
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Apply deadzone
        y = applyDeadzone(y);
        x = applyDeadzone(x);
        rx = applyDeadzone(rx);

        // Check for slow mode
        boolean slowMode = gamepad1.left_trigger > SLOW_MODE_TRIGGER_THRESHOLD;
        if (slowMode) {
            y *= SLOW_MODE_FACTOR;
            x *= SLOW_MODE_FACTOR;
            rx *= SLOW_MODE_FACTOR;
        }

        // If all inputs are zero, stop the drive
        if (Math.abs(y) < 0.01 && Math.abs(x) < 0.01 && Math.abs(rx) < 0.01) {
            drive.stop();
        } else {
//            drive.driveMecanum(y, x, rx);
        }
    }

    /**
     * Applies deadzone to a joystick input.
     * @param value The raw joystick value
     * @return The value with deadzone applied (0 if within deadzone)
     */
    private double applyDeadzone(double value) {
        if (Math.abs(value) < DEADZONE) {
            return 0.0;
        }
        return value;
    }
}