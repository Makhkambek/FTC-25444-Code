package org.firstinspires.ftc.teamcode.Controllers;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Clutch;
import org.firstinspires.ftc.teamcode.Subsystems.Push;
import org.firstinspires.ftc.teamcode.Commands.OpenPushCommand;
import org.firstinspires.ftc.teamcode.Commands.ClosePushCommand;

public class PushController {
    public PushController(Push push, Clutch clutch, Gamepad gamepad) {
        GamepadEx gamepadEx = new GamepadEx(gamepad);

        new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(new OpenPushCommand(push, clutch));

        new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(new ClosePushCommand(push, clutch));
    }
}