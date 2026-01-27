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
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
//                shooterMotor1.setPower(1.0);
                shooterMotor2.setPower(1.0);
            } else if (gamepad1.dpad_down) {
//                shooterMotor1.setPower(-1.0);
                shooterMotor2.setPower(-1.0);
            } else {
//                shooterMotor1.setPower(0.0);
                shooterMotor2.setPower(0.0);
            }
        }
    }
}
