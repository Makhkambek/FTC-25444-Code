
package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.Controllers.*;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpTest")
public class TeleOpTest extends CommandOpMode {

    private Drivetrain drive;
    private HangIntake hangIntake;
    private ArmIntake armIntake;
    private Clutch clutch;
    private Push push;
    private Vision vision;

    private DriveController driveController;
    private HangIntakeController hangIntakeController;
    private ArmIntakeController armIntakeController;
    private ClutchController clutchController;
    private PushController pushController;
    private VisionController visionController;
    private GamepadEx gamepad;

    @Override
    public void initialize() {
        //gamepad1.rumble(1.0, 1.0, 500);

        drive = new Drivetrain(hardwareMap);
        hangIntake = new HangIntake(hardwareMap);
        armIntake = new ArmIntake(hardwareMap);
        clutch = new Clutch(hardwareMap);
        push = new Push(hardwareMap);
        vision = new Vision(hardwareMap);

        driveController = new DriveController(drive, gamepad1);
        hangIntakeController = new HangIntakeController(hangIntake, gamepad2);
        armIntakeController = new ArmIntakeController(armIntake, gamepad2);
        clutchController = new ClutchController(clutch, gamepad2);
        pushController = new PushController(push, clutch, gamepad2);
        visionController = new VisionController(vision, telemetry, gamepad1);


        telemetry.addLine("TeleOp initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        telemetry.update();
    }
}
