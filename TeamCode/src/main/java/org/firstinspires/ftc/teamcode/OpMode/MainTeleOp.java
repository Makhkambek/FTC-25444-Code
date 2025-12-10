package org.firstinspires.ftc.teamcode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.*;

@TeleOp(name = "MainTeleOp", group = "Main")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        ClawSubsystem claw   = new ClawSubsystem(hardwareMap);
        ArmSubsystem arm     = new ArmSubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        SuckSubsystem suck     = new SuckSubsystem(hardwareMap);
        TailSubsystem tail     = new TailSubsystem(hardwareMap);

        final int TAIL_UP   = 600;
        final int TAIL_MID  = 300;
        final int TAIL_DOWN = 0;

        waitForStart();

        while (opModeIsActive()) {

            // DRIVETRAIN (MECANUM)
            double y  = -gamepad1.left_stick_y;    // forward/back
            double x  =  gamepad1.left_stick_x;    // strafe
            double rx =  gamepad1.right_stick_x;   // rotate

            boolean slowMode = gamepad1.left_bumper;

            drive.drive(y, x, rx, slowMode);

            // CLAW
            if (gamepad1.square)      claw.open();
            else if (gamepad1.circle) claw.close();

            // ARM
            if (gamepad1.triangle) arm.up();
            else if (gamepad1.cross) arm.down();

            // INTAKE
            if (gamepad1.left_trigger > 0.2) intake.run();
            else intake.stop();

            // SUCK MOTOR
            if (gamepad1.right_trigger > 0.3) suck.intake();
            else if (gamepad1.right_bumper)  suck.outtake();
            else suck.stop();

            // TAIL (PID POSITION CONTROL)
            if (gamepad1.dpad_up) {
                tail.setTargetPosition(TAIL_UP);
            } else if (gamepad1.dpad_right) {
                tail.setTargetPosition(TAIL_MID);
            } else if (gamepad1.dpad_down) {
                tail.setTargetPosition(TAIL_DOWN);
            }

            // PID update every loop
            tail.updatePID();

            // TELEMETRY
            telemetry.addLine("DRIVETRAIN");
            telemetry.addData("Y (forward/back)", y);
            telemetry.addData("X (strafe)", x);
            telemetry.addData("RX (rotate)", rx);
            telemetry.addData("Slow Mode", slowMode);

            telemetry.addLine("\nCLAW");
            telemetry.addData("Square = Open", gamepad1.square);
            telemetry.addData("Circle = Close", gamepad1.circle);

            telemetry.addLine("\nARM");
            telemetry.addData("Triangle = Up", gamepad1.triangle);
            telemetry.addData("X = Down", gamepad1.cross);

            telemetry.addLine("\nINTAKE");
            telemetry.addData("L2 (intake)", gamepad1.left_trigger);

            telemetry.addLine("\nSUCK");
            telemetry.addData("R2 (intake)", gamepad1.right_trigger);
            telemetry.addData("R1 (outtake)", gamepad1.right_bumper);

            telemetry.addLine("\nTAIL (PID)");
            telemetry.addData("Target Position", tail.getTargetPosition());
            telemetry.addData("Current Position", tail.getCurrentPosition());

            telemetry.update();
        }
    }
}
