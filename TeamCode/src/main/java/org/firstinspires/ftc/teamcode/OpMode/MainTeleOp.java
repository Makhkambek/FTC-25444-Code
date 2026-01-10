package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.*;
import org.firstinspires.ftc.teamcode.Controller.*;
import org.firstinspires.ftc.teamcode.Command.ShootCommand;

@TeleOp(name = "TeleOp_Main")
public class MainTeleOp extends OpMode {

    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private GateServoSubsystem gateServoSubsystem;
    private DriveController driveController;
    private IntakeController intakeController;
    private ShooterController shooterController;

    private ShootCommand shootCommand;


    @Override
    public void init() {
        driveSubsystem = new DriveSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);

        driveController = new DriveController(driveSubsystem);
        intakeController = new IntakeController(intakeSubsystem);

        shootCommand = new ShootCommand(shooterSubsystem, gateServoSubsystem, intakeSubsystem);
        shooterController = new ShooterController(shootCommand);
    }

    @Override
    public void loop() {
        driveController.update(gamepad1);
        intakeController.update(gamepad1);
    }

    @Override
    public void stop() {
        driveSubsystem.stop();
    }

}
