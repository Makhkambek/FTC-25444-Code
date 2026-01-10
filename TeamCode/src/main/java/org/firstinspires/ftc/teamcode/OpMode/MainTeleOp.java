package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Controller.DriveController;
import org.firstinspires.ftc.teamcode.Controller.IntakeController;

@TeleOp(name = "TeleOp_Main")
public class MainTeleOp extends OpMode {

    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private DriveController driveController;
    private IntakeController intakeController;

    @Override
    public void init() {
        driveSubsystem = new DriveSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        driveController = new DriveController(driveSubsystem);
        intakeController = new IntakeController(intakeSubsystem);
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
