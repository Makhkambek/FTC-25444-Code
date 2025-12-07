package org.firstinspires.ftc.teamcode.Opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Controllers.LiftController;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "teleOp")
public class teleOp extends CommandOpMode {

    private Lift lift;
    private LiftController liftController;
    private GamepadEx gamepadEx;

    @Override
    public void initialize() {
        lift = new Lift(hardwareMap);

        gamepadEx = new GamepadEx(gamepad1);

        liftController = new LiftController(lift, gamepadEx);

        telemetry.addLine("teleOp Initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("Lift Position", lift.getCurrentPosition());
        telemetry.update();
    }
}
