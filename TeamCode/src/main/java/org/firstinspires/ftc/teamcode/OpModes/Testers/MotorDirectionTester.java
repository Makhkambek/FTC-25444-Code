package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="[TEST] Motor Direction", group="Testers")
public class MotorDirectionTester extends LinearOpMode {

    private DcMotor shooterMotor1;
    private DcMotor shooterMotor2;

    @Override
    public void runOpMode() {
        shooterMotor1 = hardwareMap.get(DcMotor.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");

        // Motor2 в REVERSE для синхронного вращения
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("=== MOTOR DIRECTION TESTER ===");
        telemetry.addLine("D-pad Up: Both motors forward");
        telemetry.addLine("D-pad Down: Both motors backward");
        telemetry.addLine();
        telemetry.addLine("Check if motors spin TOWARDS each other");
        telemetry.addLine("(creating tunnel for ball)");
        telemetry.addLine();
        telemetry.addLine("If they spin AWAY from each other:");
        telemetry.addLine("- Physically swap wires on Motor1");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
//                shooterMotor1.setPower(1.0);
                shooterMotor2.setPower(1.0);
                telemetry.addData("Status", "FORWARD");
            } else if (gamepad1.dpad_down) {
//                shooterMotor1.setPower(-1.0);
                shooterMotor2.setPower(-1.0);
                telemetry.addData("Status", "BACKWARD");
            } else {
//                shooterMotor1.setPower(0.0);
                shooterMotor2.setPower(0.0);
                telemetry.addData("Status", "STOPPED");
            }

            telemetry.addLine();
            telemetry.addLine("Motors should spin TOWARDS each other!");
            telemetry.update();
        }
    }
}
