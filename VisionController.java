package org.firstinspires.ftc.teamcode.Controllers;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.command.CommandScheduler;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Commands.DetectTagCommand;


public class VisionController {
    public VisionController(Vision vision, Telemetry telemetry, Gamepad gamepad) {
        GamepadEx gamepadEx = new GamepadEx(gamepad);

        DetectTagCommand detectTagCommand = new DetectTagCommand(vision, telemetry, gamepad);

        new Trigger(() -> gamepadEx.getButton(GamepadKeys.Button.X))
                .whenActive(() -> CommandScheduler.getInstance().schedule(detectTagCommand))
                .whenInactive(() -> CommandScheduler.getInstance().cancel(detectTagCommand));
    }
}