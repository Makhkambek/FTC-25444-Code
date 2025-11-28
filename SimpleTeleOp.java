package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Simple TeleOp", group = "TeleOp")
public class SimpleTeleOp extends OpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    @Override
    public void init() {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Speed control with triggers (RT = fast, LT = slow)
        double speedMultiplier = 1.0;
        if (gamepad1.right_trigger > 0.1) {
            speedMultiplier = 0.3; // Slow mo

            speedMultiplier = 1.5; // Turbo mode (capped at 1.0 after normalization)
        }
        
        // Get joystick values
        double drive = -gamepad1.left_stick_y;

        double turn = gamepad1.right_stick_x;
        
        // Calculate motor powers for mecanum drive
        double leftFrontPower = (drive + strafe + turn) * speedM

        double leftBackPower = (drive - strafe + turn) * speedMultiplier;
        double rightBackPower = (drive + strafe - turn) * speedMultiplier;
        
        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        m

        
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;

        }
        
        // Set motor powers
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        
        // Display telemetry
        String speedMode = "NORMAL";
        if (gamepad1.right_trigger > 0.1) speedMode = "SLOW";
        if (gamepad1.left_trigger > 0.1) speedMode = "TURBO";
        
        telemetry.addData("Speed Mode", speedMode);
        telemetry.addData("Left Front Power", "%.2f", leftFrontPower);
        telemetry.addData("Right Front Power", "%.2f", rightFrontPower);
        telemetry.addData("Left Back Power", "%.2f", leftBackPower);
        telemetry.addData("Right Back Power", "%.2f", rightBackPower);
        telemetry.update();
    }
}
