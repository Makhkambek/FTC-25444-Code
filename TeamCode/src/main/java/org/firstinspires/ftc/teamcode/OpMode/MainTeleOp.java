package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.*;

@TeleOp(name = "MainTeleOp", group = "Main")
public class MainTeleOp extends LinearOpMode {

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
            // DRIVETRAIN (MECANUM)
            // -------------------
            double y  = -gamepad1.left_stick_y;    // forward/back
            double x  =  gamepad1.left_stick_x;    // strafe left/right
            double rx =  gamepad1.right_stick_x;   // rotate

            boolean slowMode = gamepad1.left_bumper;

            drive.drive(y, x, rx, slowMode);


            // -------------------
            // CLAW
            // -------------------
            if (gamepad1.square)  claw.open();
            else if (gamepad1.circle) claw.close();


            // -------------------
            // ARM (push servos)
            // -------------------
            if (gamepad1.triangle) arm.up();
            else if (gamepad1.cross) arm.down();


            // -------------------
            // INTAKE (CRServos)
            // -------------------
            if (gamepad1.left_trigger > 0.2) intake.run();
            else intake.stop();


            // -------------------
            // SUCK MOTOR
            // -------------------
            if (gamepad1.right_trigger > 0.3) suck.intake();
            else if (gamepad1.right_bumper) suck.outtake();
            else suck.stop();


            // -------------------
            // TAIL MOTOR
            // -------------------
            if (gamepad1.dpad_right) tail.up();
            else if (gamepad1.dpad_left) tail.down();
            else tail.stop();


            // -------------------
            // TELEMETRY
            // -------------------
            telemetry.addLine("=== DRIVETRAIN ===");
            telemetry.addData("Y (forward/back)", y);
            telemetry.addData("X (strafe)", x);
            telemetry.addData("RX (rotate)", rx);
            telemetry.addData("Slow Mode", slowMode ? "ON" : "OFF");

            telemetry.addLine("\n=== CLAW ===");
            telemetry.addData("Square = Open", gamepad1.square);
            telemetry.addData("Circle = Close", gamepad1.circle);

            telemetry.addLine("\n=== ARM ===");
            telemetry.addData("Triangle = Up", gamepad1.triangle);
            telemetry.addData("X = Down", gamepad1.cross);

            telemetry.addLine("\n=== INTAKE ===");
            telemetry.addData("L2 (intake)", gamepad1.left_trigger);

            telemetry.addLine("\n=== SUCK ===");
            telemetry.addData("R2 (intake)", gamepad1.right_trigger);
            telemetry.addData("R1 (outtake)", gamepad1.right_bumper);

            telemetry.addLine("\n=== TAIL ===");
            telemetry.addData("Dpad Right = Up", gamepad1.dpad_right);
            telemetry.addData("Dpad Left = Down", gamepad1.dpad_left);

            telemetry.update();
        }
    }
}
