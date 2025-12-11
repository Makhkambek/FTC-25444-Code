package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.*;

@TeleOp(name = "MainTeleOp", group = "Main")
public class MainTeleOp extends LinearOpMode {

    private static final int UP_POSITION = 1300;
    private static final int DOWN_POSITION = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize all subsystems
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        ClawSubsystem claw = new ClawSubsystem(hardwareMap);
        ArmSubsystem arm = new ArmSubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        SuckSubsystem suck = new SuckSubsystem(hardwareMap);
        TailSubsystem tail = new TailSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // -------------------
            // DRIVETRAIN
            // -------------------
            double driveInput = -gamepad1.left_stick_y;
            double turnInput = gamepad1.right_stick_x;

            // Deadzone
            if (Math.abs(driveInput) < 0.07) driveInput = 0;
            if (Math.abs(turnInput) < 0.07) turnInput = 0;

            // Smooth control
            driveInput = Math.pow(driveInput, 3);
            turnInput = Math.pow(turnInput, 3);

            boolean slowMode = gamepad1.left_bumper;

            double strafeInput = 0; // if you want pure arcade (no strafe)
            if (slowMode) {
                drive.drive(driveInput * 0.5, strafeInput * 0.5, turnInput * 0.5);
            } else {
                drive.drive(driveInput, strafeInput, turnInput);
            }



            // -------------------
            // CLAW
            // Square → open
            // Circle → close
            // -------------------
            if (gamepad1.square) claw.open();
            else if (gamepad1.circle) claw.close();


            // -------------------
            // ARM (push_1, push_2)
            // Triangle → up
            // X → down
            // -------------------
            if (gamepad1.triangle) arm.up();
            else if (gamepad1.cross) arm.down();


            // -------------------
            // INTAKE (CRServos)
            // L2 → run intake
            // Release → stop
            // -------------------
            if (gamepad1.left_trigger > 0.2) intake.run();
            else intake.stop();


            // -------------------
            // SUCK MOTOR
            // R2 → intake
            // R1 → outtake
            // -------------------
            if (gamepad1.right_trigger > 0.3) suck.intake();
            else if (gamepad1.right_bumper) suck.outtake();
            else suck.stop();


            // -------------------
            // TAIL MOTOR
            // Dpad Right → up
            // Dpad Left  → down
            // -------------------
            if (gamepad1.dpad_right) {
                tail.setTargetPosition(UP_POSITION);    // replace with your ticks
            } else if (gamepad1.dpad_left) {
                tail.setTargetPosition(DOWN_POSITION);  // replace with your ticks
            }
            tail.updatePID();  // ALWAYS call this each loop


            // -------------------
            // TELEMETRY
            // -------------------
            telemetry.addLine("=== DRIVETRAIN ===");
            telemetry.addData("Drive", driveInput);
            telemetry.addData("Turn", turnInput);
            telemetry.addData("Slow Mode", slowMode ? "ON" : "OFF");

            telemetry.addLine("\n=== CLAW ===");
            telemetry.addData("Square", gamepad1.square);
            telemetry.addData("Circle", gamepad1.circle);

            telemetry.addLine("\n=== ARM ===");
            telemetry.addData("Triangle", gamepad1.triangle);
            telemetry.addData("X", gamepad1.cross);

            telemetry.addLine("\n=== INTAKE ===");
            telemetry.addData("L2", gamepad1.left_trigger);

            telemetry.addLine("\n=== SUCK ===");
            telemetry.addData("R2", gamepad1.right_trigger);
            telemetry.addData("R1", gamepad1.right_bumper);

            telemetry.addLine("\n=== TAIL ===");
            telemetry.addData("Dpad Right", gamepad1.dpad_right);
            telemetry.addData("Dpad Left", gamepad1.dpad_left);

            telemetry.update();
        }
    }
}