package org.firstinspires.ftc.teamcode.Drivetrain_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorIdentificationTest")
public class test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor lf = hardwareMap.dcMotor.get("lf");
        DcMotor rf = hardwareMap.dcMotor.get("rf");
        DcMotor lr = hardwareMap.dcMotor.get("lr");
        DcMotor rr = hardwareMap.dcMotor.get("rr");

        waitForStart();

        // Test LF
        telemetry.addData("Testing", "LF - LEFT FRONT");
        telemetry.update();
        lf.setPower(0.4);
        sleep(1500);
        lf.setPower(0);

        // Test RF
        telemetry.addData("Testing", "RF - RIGHT FRONT");
        telemetry.update();
        rf.setPower(0.4);
        sleep(1500);
        rf.setPower(0);

        // Test LR
        telemetry.addData("Testing", "LR - LEFT REAR");
        telemetry.update();
        lr.setPower(0.4);
        sleep(1500);
        lr.setPower(0);

        // Test RR
        telemetry.addData("Testing", "RR - RIGHT REAR");
        telemetry.update();
        rr.setPower(0.4);
        sleep(1500);
        rr.setPower(0);

        telemetry.addData("Done", "All motors tested");
        telemetry.update();
        sleep(1000);
    }
}
