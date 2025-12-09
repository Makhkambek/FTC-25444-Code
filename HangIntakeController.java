package org.firstinspires.ftc.teamcode.Controllers;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.HangIntake;
import org.firstinspires.ftc.teamcode.Commands.HangIntakeCommand;

public class HangIntakeController {
    private final GamepadEx gamepadEx;

    public HangIntakeController(HangIntake hang, Gamepad gamepad) {
        this.gamepadEx = new GamepadEx(gamepad);

        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(new HangIntakeCommand(hang, 1.0));

        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whileHeld(new HangIntakeCommand(hang, -1.0));
    }
}