package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;

@TeleOp(name = "ClawControl_TeleOp")
public class ClawControl_TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        ClawSubsystem claw = new ClawSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // Rectangle button (square) → open
            if (gamepad1.square) {
                claw.open();
            }

            // Circle button → close
            if (gamepad1.circle) {
                claw.close();
            }

            telemetry.addData("Claw State",
                    gamepad1.square ? "OPEN" :
                            gamepad1.circle ? "CLOSE" : "IDLE");

            telemetry.update();
        }
    }
}