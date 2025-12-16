package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drivetrain extends SubsystemBase {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private BNO055IMU imu;

    // HEADING
    private double targetHeading;
    private static final double kP = 0.015;

    public Drivetrain(HardwareMap hardwareMap) {

        leftFront  = hardwareMap.dcMotor.get("leftFront");
        leftBack   = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack  = hardwareMap.dcMotor.get("rightBack");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(params);

        targetHeading = getHeading();
        stop();
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation();
        return angles.firstAngle;
    }

    public void resetHeading() {
        targetHeading = getHeading();
    }

    public void arcadeDrive(double forward, double turn) {

        double error = targetHeading - getHeading();
        double correction = error * kP;

        if (Math.abs(turn) > 0.05) {
            correction = 0;
            targetHeading = getHeading();
        }

        double leftPower  = forward + correction;
        double rightPower = forward - correction;

        leftPower  = Math.max(-1, Math.min(1, leftPower));
        rightPower = Math.max(-1, Math.min(1, rightPower));

        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);
    }

    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
