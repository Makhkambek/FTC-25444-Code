package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class test extends OpMode {

    private DcMotor leftFront, rightFront, leftRear, rightRear;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("", "");
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("Left Stick", "Drive + Strafe");
        telemetry.addData("Right Stick X", "Rotate");
        telemetry.addData("Right Trigger", "Slow Mode (30%)");
        telemetry.update();
    }

    @Override
    public void loop() {
        double slowModeFactor = gamepad1.right_trigger > 0.1 ? 0.3 : 1.0;

        double y = -gamepad1.left_stick_y * slowModeFactor;
        double x = gamepad1.left_stick_x * slowModeFactor;
        double rx = gamepad1.right_stick_x * slowModeFactor;

        double frontLeftPower = y + x + rx;
        double backLeftPower = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower = y + x - rx;

        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backLeftPower),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);

        // Телеметрия
        telemetry.addData("=== DRIVE ===", "");
        telemetry.addData("Slow Mode", gamepad1.right_trigger > 0.1 ? "ON (30%)" : "OFF (100%)");
        telemetry.addData("", "");
        telemetry.addData("=== INPUTS ===", "");
        telemetry.addData("Drive (Y)", "%.2f", y);
        telemetry.addData("Strafe (X)", "%.2f", x);
        telemetry.addData("Rotate", "%.2f", rx);
        telemetry.addData("", "");
        telemetry.addData("=== MOTORS ===", "");
        telemetry.addData("FL / FR", "%.2f / %.2f", frontLeftPower, frontRightPower);
        telemetry.addData("BL / BR", "%.2f / %.2f", backLeftPower, backRightPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}