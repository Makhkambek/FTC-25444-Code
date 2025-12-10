package org.firstinspires.ftc.teamcode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

@TeleOp(name="DriveSubsystem_TeleOp", group="Main")
public class DriveSubsystem_TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        DriveSubsystem drive = new DriveSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            boolean slow = gamepad1.left_bumper;

            drive.drive(y, x, rx, slow);

            telemetry.addData("Slow Mode", slow ? "ON" : "OFF");
            telemetry.update();
        }
    }
}
