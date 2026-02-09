package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller.DriveController;
import org.firstinspires.ftc.teamcode.Controller.IntakeController;
import org.firstinspires.ftc.teamcode.Controller.ShootingController;
import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.VisionSubsystem;

@TeleOp(name = "Main TeleOp - BLUE", group = "Main")
public class BlueMainOpMode extends LinearOpMode {

    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private VisionSubsystem visionSubsystem;
    private ShooterSubsystem shooterSubsystem;

    private DriveController driveController;
    private IntakeController intakeController;
    private ShootingController shootingController;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing subsystems for BLUE alliance...");
        telemetry.update();

        driveSubsystem = new DriveSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        visionSubsystem = new VisionSubsystem(hardwareMap, VisionSubsystem.AllianceColor.BLUE);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);

        driveController = new DriveController(driveSubsystem);
        intakeController = new IntakeController(intakeSubsystem);
        shootingController = new ShootingController(
                shooterSubsystem,
                intakeSubsystem,
                visionSubsystem,
                gamepad2,  // Only gamepad2 for shooting
                telemetry
        );

        telemetry.addLine("Initialization complete!");
        telemetry.addLine("Ready to start - Alliance: BLUE");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driveController.update(gamepad1);

            // Only update intake if gate sequence is NOT active (avoid conflict)
            if (!shootingController.isGateSequenceActive()) {
                intakeController.update(gamepad1);
            }

            shootingController.update();

            telemetry.update();
        }

        if (visionSubsystem != null) {
            visionSubsystem.close();
        }
    }
}