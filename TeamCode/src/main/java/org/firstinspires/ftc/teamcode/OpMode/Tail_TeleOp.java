package org.firstinspires.ftc.teamcode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.TailSubsystem;

@TeleOp(name = "Tail_TeleOp")
public class Tail_TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        TailSubsystem tail = new TailSubsystem(hardwareMap);

        final int TAIL_UP = 600;
        final int TAIL_MID = 300;
        final int TAIL_DOWN = 0;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                tail.setTargetPosition(TAIL_UP);
            }
            else if (gamepad1.dpad_right) {
                tail.setTargetPosition(TAIL_MID);
            }
            else if (gamepad1.dpad_down) {
                tail.setTargetPosition(TAIL_DOWN);
            }

            // PID must update every loop
            tail.updatePID();

            telemetry.addData("Target Position", tail.getTargetPosition());
            telemetry.addData("Current Position", tail.getCurrentPosition());
            telemetry.update();
        }
    }
}