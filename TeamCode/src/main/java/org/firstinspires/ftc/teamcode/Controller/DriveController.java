package org.firstinspires.ftc.teamcode.Controller;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;

public class DriveController {

    private final DriveSubsystem drive;

    private static final double DEADBAND = 0.05;
    private static final double NORMAL_SPEED = 1.0;
    private static final double SLOW_SPEED = 0.35;

    private boolean slowMode = false;
    private boolean lastSlowButton = false;

    public DriveController(DriveSubsystem drive) {
        this.drive = drive;
    }

    public void update(Gamepad gamepad) {

        // Toggle slow mode (left bumper)
        boolean slowButton = gamepad.left_bumper;
        if (slowButton && !lastSlowButton) {
            slowMode = !slowMode;
        }
        lastSlowButton = slowButton;

        drive.setSpeedMultiplier(slowMode ? SLOW_SPEED : NORMAL_SPEED);

        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;

        y = applyDeadband(y);
        x = applyDeadband(x);
        turn = applyDeadband(turn);

        drive.drive(y, x, turn);
    }

    private double applyDeadband(double value) {
        return Math.abs(value) > DEADBAND ? value : 0;
    }
}
