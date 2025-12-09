package org.firstinspires.ftc.teamcode.Controllers;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Commands.DriveCommand;

public class DriveController {

    private final Drivetrain drive;
    private final GamepadEx gamepadEx;

    public DriveController(Drivetrain drive, Gamepad gamepad) {
        this.drive = drive;
        this.gamepadEx = new GamepadEx(gamepad);

        setupControls();
    }

    private void setupControls() {
        drive.setDefaultCommand(
                new DriveCommand(
                        drive,
                        () -> { // f/b
                            double y = -gamepadEx.getLeftY();
                            return Math.abs(y) > 0.05 ? y : 0.0;
                        },
                        () -> { // turning
                            double x = gamepadEx.getRightX();
                            return Math.abs(x) > 0.05 ? x : 0.0;
                        }
                )
        );
    }
}