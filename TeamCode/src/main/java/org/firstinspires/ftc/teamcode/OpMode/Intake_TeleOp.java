package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

@TeleOp(name = "Intake_TeleOp")
public class Intake_TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // Hold L2 to run intake
            if (gamepad1.left_trigger > 0.2) {
                intake.run();
            } else {
                intake.stop();
            }

            telemetry.addData("Intake",
                    gamepad1.left_trigger > 0.2 ? "RUNNING" : "STOPPED");

            telemetry.update();
        }
    }
}
