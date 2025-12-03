package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.TailSubsystem;

@TeleOp(name = "Tail_TeleOp")
public class Tail_TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        TailSubsystem tail = new TailSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_right) {
                tail.up();
            } else if (gamepad1.dpad_left) {
                tail.down();
            } else {
                tail.stop();
            }

            telemetry.addData("Right", gamepad1.dpad_right);
            telemetry.addData("Left", gamepad1.dpad_left);
            telemetry.update();
        }
    }
}
