package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Push;
import org.firstinspires.ftc.teamcode.Subsystems.Clutch;

public class OpenPushCommand extends SequentialCommandGroup {
    public OpenPushCommand(Push push, Clutch clutch) {
        addRequirements(push, clutch);

        addCommands(
                new InstantCommand(() -> clutch.setPosition(0.9), clutch),
                new WaitCommand(500),

                new InstantCommand(push::open, push),
                new WaitCommand(700),

                new InstantCommand(clutch::close, clutch),
                new WaitCommand(500)
        );
    }
}