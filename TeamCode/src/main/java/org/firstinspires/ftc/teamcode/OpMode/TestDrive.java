package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Controller.DriveController;
import org.firstinspires.ftc.teamcode.Controller.IntakeController;
import org.firstinspires.ftc.teamcode.Subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;

@TeleOp(name = "Test Drive", group = "Test")
public class TestDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        DriveController driveController = new DriveController(drive);
        IntakeController intakeController = new IntakeController(intake);

        telemetry.addLine("Test Drive Ready - Press START");
        telemetry.addLine("Left Stick Y:   Forward / Backward");
        telemetry.addLine("Right Stick X:  Strafe Left / Right");
        telemetry.addLine("Left Stick X:   Turn Left / Right");
        telemetry.addLine("Left Trigger:   Slow Mode");
        telemetry.addLine("Right Trigger:  Intake In");
        telemetry.addLine("Right Bumper:   Intake Out");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driveController.update(gamepad1);
            intakeController.update(gamepad1);

            telemetry.addData("Left Trigger Raw", "%.2f", gamepad1.left_trigger);
            telemetry.addData("Slow Mode", gamepad1.left_trigger > 0.1 ? "ON" : "OFF");
            telemetry.addData("Left Stick Y", "%.2f", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X", "%.2f", gamepad1.right_stick_x);
            telemetry.addData("Left Stick X", "%.2f", gamepad1.left_stick_x);
            telemetry.addLine();
            telemetry.addData("LF encoder", drive.getLeftFrontEncoder());
            telemetry.addData("RF encoder", drive.getRightFrontEncoder());
            telemetry.addData("LR encoder", drive.getLeftRearEncoder());
            telemetry.addData("RR encoder", drive.getRightRearEncoder());
            telemetry.update();
        }

        drive.stop();
        intake.stop();
    }
}
