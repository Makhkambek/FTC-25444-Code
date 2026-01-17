package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain {
    private DcMotor leftFront, rightFront, leftRear, rightRear;
    private HeadingController headingController;
    private boolean wasLeftBumperPressed = false;

    public DriveController(HardwareMap hardwareMap, Telemetry telemetry) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        headingController = new HeadingController(hardwareMap);
    }

    public void drive(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        headingController.update(telemetry);

        if (gamepad1.left_bumper && !wasLeftBumperPressed) {
            headingController.lockHeading();
            wasLeftBumperPressed = true;
            telemetry.addData("Heading Action", "Lock Heading");
        } else if (!gamepad1.left_bumper && wasLeftBumperPressed) {
            headingController.unlockHeading();
            wasLeftBumperPressed = false;
            telemetry.addData("Heading Action", "Unlock Heading");
        }

        double slowModeFactor = gamepad1.right_trigger > 0.1 ? 0.3 : 1.0;
        double y = -gamepad1.left_stick_y * slowModeFactor;
        double x = gamepad1.left_stick_x * slowModeFactor;
        double rx;

        if (gamepad1.right_bumper) {
            rx = headingController.calculateTurnPower();
        } else {
            rx = gamepad1.right_stick_x * slowModeFactor;
        }

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

//        telemetry.addData("Front Left Power", frontLeftPower);
//        telemetry.addData("Back Left Power", backLeftPower);
//        telemetry.addData("Front Right Power", frontRightPower);
//        telemetry.addData("Back Right Power", backRightPower);
//        telemetry.addData("X Input", x);
    }
}