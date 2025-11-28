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
        // Get joystick values
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        
        // Calculate motor powers for mecanum drive
        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower = drive - strafe + turn;
        double rightBackPower = drive + strafe - turn;
        
        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightBackPower));
        
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }
        
        // Set motor powers
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        
        // Display telemetry
        telemetry.addData("Left Front Power", leftFrontPower);
        telemetry.addData("Right Front Power", rightFrontPower);
        telemetry.addData("Left Back Power", leftBackPower);
        telemetry.addData("Right Back Power", rightBackPower);
        telemetry.update();
    }
}
