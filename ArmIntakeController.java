package org.firstinspires.ftc.teamcode.Controllers;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Subsystems.ArmIntake;
import org.firstinspires.ftc.teamcode.Commands.ArmIntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.MoveDoorCommand;

public class ArmIntakeController {

    public ArmIntakeController(ArmIntake armIntake, Gamepad gamepad) {
        GamepadEx gamepadEx = new GamepadEx(gamepad);

        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(new ArmIntakeCommand(armIntake, 1.0));
        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(new ArmIntakeCommand(armIntake, -1.0));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new MoveDoorCommand(armIntake, 0));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new MoveDoorCommand(armIntake, 150));
    }
}