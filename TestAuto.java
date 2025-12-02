package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TestAuto", group = "Auto")
public class TestAuto extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        driveForward(0.5, 2000);
        sleep(500);
        strafeRight(0.5, 1500);
        sleep(500);
        turnLeft(0.4, 1000);
        sleep(500);
        driveBackward(0.5, 1000);
    }

    public void driveForward(double power, long time) {
        setMotorPowers(power, power, power, power);
        sleep(time);
        stopMotors();
    }

    public void driveBackward(double power, long time) {
        setMotorPowers(-power, -power, -power, -power);
        sleep(time);
        stopMotors();
    }

    public void strafeRight(double power, long time) {
        setMotorPowers(power, -power, -power, power);
        sleep(time);
        stopMotors();
    }

    public void strafeLeft(double power, long time) {
        setMotorPowers(-power, power, power, -power);
        sleep(time);
        stopMotors();
    }

    public void turnLeft(double power, long time) {
        setMotorPowers(-power, power, -power, power);
        sleep(time);
        stopMotors();
    }

    public void turnRight(double power, long time) {
        setMotorPowers(power, -power, power, -power);
        sleep(time);
        stopMotors();
    }

    public void setMotorPowers(double lf, double rf, double lb, double rb) {
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }

    public void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }
}
