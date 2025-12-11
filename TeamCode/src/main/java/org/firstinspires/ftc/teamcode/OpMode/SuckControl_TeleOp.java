package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.SuckSubsystem;

@TeleOp(name = "SuckControl_TeleOp")
public class SuckControl_TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SuckSubsystem suck = new SuckSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // R2 hold → intake
            if (gamepad1.right_trigger > 0.3) {
                suck.intake();
            }

            // R1 hold → outtake
            else if (gamepad1.right_bumper) {
                suck.outtake();
            }

            // None → stop
            else {
                suck.stop();
            }

            telemetry.addData("Suck State",
                    gamepad1.right_trigger > 0.3 ? "INTAKE" :
                            gamepad1.right_bumper ? "OUTTAKE" : "STOP");

            telemetry.update();
        }
    }
}