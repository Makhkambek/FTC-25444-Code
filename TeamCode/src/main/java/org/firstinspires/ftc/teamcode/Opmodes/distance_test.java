package org.firstinspires.ftc.teamcode.Opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Commands.GetDistance;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;

@TeleOp(name = "AprilTag Distance (Command Based)")
public class distance_test extends CommandOpMode {

    private Vision vision;

    @Override
    public void initialize() {
        vision = new Vision(hardwareMap);

        schedule(new GetDistance(vision, telemetry));

        telemetry.addLine(">> Initialized and Running");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
