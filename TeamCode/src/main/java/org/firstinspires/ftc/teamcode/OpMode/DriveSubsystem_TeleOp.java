package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

@TeleOp(name = "DriveSubsystem_TeleOp")
public class DriveSubsystem_TeleOp extends OpMode {

    private DriveSubsystem drive;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;  // forward/back
        double x = gamepad1.left_stick_x;   // strafe
        double turn = gamepad1.right_stick_x; // rotation

        drive.drive(y, x, turn);
    }
}