package org.firstinspires.ftc.teamcode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

@TeleOp(name = "ArmControl_TeleOp")
public class ArmControl_TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        ArmSubsystem arm = new ArmSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.triangle) {
                arm.up();
            }
            if (gamepad1.cross) {
                arm.down();
            }

            telemetry.addData("Arm State",
                    gamepad1.triangle ? "UP" :
                            gamepad1.cross ? "DOWN" : "IDLE");

            telemetry.update();
        }
    }
}
