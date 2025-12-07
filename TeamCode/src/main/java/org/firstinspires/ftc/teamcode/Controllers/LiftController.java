package org.firstinspires.ftc.teamcode.Controllers;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Commands.LiftToHeightCommand;

public class LiftController {

    public LiftController(Lift lift, GamepadEx gamepadEx) {

        new Trigger(() -> gamepadEx.getButton(GamepadKeys.Button.A))
                .whenActive(new LiftToHeightCommand(lift, Lift.GND));
        new Trigger(() -> gamepadEx.getButton(GamepadKeys.Button.B))
                .whenActive(new LiftToHeightCommand(lift, Lift.LOW));
        new Trigger(() -> gamepadEx.getButton(GamepadKeys.Button.X))
                .whenActive(new LiftToHeightCommand(lift, Lift.MID));
        new Trigger(() -> gamepadEx.getButton(GamepadKeys.Button.Y))
                .whenActive(new LiftToHeightCommand(lift, Lift.HIGH));

    }
}
