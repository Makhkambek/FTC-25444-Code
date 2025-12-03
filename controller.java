package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "tutorial")// file name
public class tutorial extends OpMode {
    DcMotor motor1;// motor name
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;

    @Override
    public void init() {
        // motor1 = hardwareMap.get (DcMotor.class, "ArmIntakeMotor");
        // motor2 = hardwareMap.get(DcMotor.class, "leftMotor");
        // motor3 = hardwareMap.get(DcMotor.class, "leftFront");
        // motor4 = hardwareMap.get(DcMotor.class, "leftBack");
        telemetry.addData("Hardware: ", "Initialized");

    }

    @Override
    public void loop() {
        float x = gamepad1.left_stick_x;
        float y = gamepad1.left_stick_y;
        //  telemetry.addData(String.valueOf(x), "x");
        //  telemetry.addData(String.valueOf(y), "y");
        if (y == -1){
            if (0.6> x & x>-0.6){
                telemetry.addData("Movement", "forward");
                // motor1.setPower(1);
                // motor2.setPower(1);
                // motor3.setPower(1);
                //motor4.setPower(1);
            }}
        if (y==1){
            if (0.6> x & x>-0.6){
                telemetry.addData("Movement", "back");
                // motor1.setPower(-1);
                // motor2.setPower(-1);
                // motor3.setPower(-1);
                //motor4.setPower(-1);
            }}
        if (x==-1){
            if(y <0.5 & y>-1){
                telemetry.addData("Movement", "left");
                // motor1.setPower(1);
                // motor2.setPower(1);
                // motor3.setPower(-1);
                //motor4.setPower(-1);
            }}
        if (x==1){
            if (y>-0.9 & y< 0.4) {
                telemetry.addData("Movement", "right");
                // motor1.setPower(-1);
                // motor2.setPower(-1);
                // motor3.setPower(1);
                //motor4.setPower(1);
            }

        }
    }
}