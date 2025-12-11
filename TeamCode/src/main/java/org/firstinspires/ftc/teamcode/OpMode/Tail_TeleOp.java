package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.TailSubsystem;

@TeleOp(name = "Tail_TeleOp")
public class Tail_TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        TailSubsystem tail = new TailSubsystem(hardwareMap);

        // You can adjust these depending on robot limits
        final int TAIL_UP = 900;
        final int TAIL_DOWN = 0;

        waitForStart();

        while (opModeIsActive()) {

            // === Input controls exactly like your original ===
            if (gamepad1.dpad_right) {
                tail.setTargetPosition(TAIL_UP);
            }
            else if (gamepad1.dpad_left) {
                tail.setTargetPosition(TAIL_DOWN);
            }

            // === PID loop (required every cycle!) ===
            tail.updatePID();

            // === Telemetry ===
            telemetry.addData("Target", tail.getTargetPosition());
            telemetry.addData("Current", tail.getCurrentPosition());
            telemetry.addData("Error", tail.getTargetPosition() - tail.getCurrentPosition());
            telemetry.update();

        }
    }
}
