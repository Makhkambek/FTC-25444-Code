package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller.IntakeController;
import org.firstinspires.ftc.teamcode.Controller.ShootingController;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.VisionSubsystem;

@TeleOp(name = "Main TeleOp - BLUE", group = "Main")
public class BlueMainOpMode extends LinearOpMode {

    private IntakeSubsystem intakeSubsystem;
    private VisionSubsystem visionSubsystem;
    private ShooterSubsystem shooterSubsystem;

    private IntakeController intakeController;
    private ShootingController shootingController;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing subsystems for BLUE alliance...");
        telemetry.update();

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        visionSubsystem = new VisionSubsystem(hardwareMap, VisionSubsystem.AllianceColor.BLUE);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);

        intakeController = new IntakeController(intakeSubsystem);
        shootingController = new ShootingController(
                shooterSubsystem,
                intakeSubsystem,
                visionSubsystem,
                gamepad1,
                gamepad2,
                telemetry
        );

        telemetry.addLine("Initialization complete!");
        telemetry.addLine("Ready to start - Alliance: BLUE");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            intakeController.update(gamepad1);
            shootingController.update();

            telemetry.update();
        }

        if (visionSubsystem != null) {
            visionSubsystem.close();
        }
    }
}