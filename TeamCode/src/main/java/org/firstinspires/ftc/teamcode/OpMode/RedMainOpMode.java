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

/**
 * RedMainOpMode: Main TeleOp for RED alliance.
 *
 * Gamepad 1 Controls:
 * - Left stick: Drive (Y = forward/back, X = strafe)
 * - Right stick X: Turn
 * - Left trigger: Slow mode
 * - Right trigger: Intake in
 * - Right bumper: Outtake
 *
 * Gamepad 2 Controls:
 * - Right stick X: Manual turret control (when not in auto-aim)
 * - Right trigger: Enter target mode
 * - Left trigger: Shoot (when locked on)
 */
@TeleOp(name = "Main TeleOp - RED", group = "Main")
public class RedMainOpMode extends LinearOpMode {

    // Subsystems
    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private VisionSubsystem visionSubsystem;
    private ShooterSubsystem shooterSubsystem;

    // Controllers
    private DriveController driveController;
    private IntakeController intakeController;
    private ShootingController shootingController;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        telemetry.addLine("Initializing subsystems for RED alliance...");
        telemetry.update();

        driveSubsystem = new DriveSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        visionSubsystem = new VisionSubsystem(hardwareMap, VisionSubsystem.AllianceColor.RED);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);

        // Initialize controllers
        driveController = new DriveController(driveSubsystem);
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
        telemetry.addLine("Ready to start - Alliance: RED");
        telemetry.update();

        // Wait for start
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Update all controllers
            driveController.update(gamepad1);
            intakeController.update(gamepad1);
            shootingController.update();

            // Update telemetry
            telemetry.update();
        }

        // Cleanup
        if (visionSubsystem != null) {
            visionSubsystem.close();
        }
    }
}