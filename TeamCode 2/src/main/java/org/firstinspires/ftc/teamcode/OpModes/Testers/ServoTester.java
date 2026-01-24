package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="[TEST] Servo Tester", group="Testers")
public class ServoTester extends LinearOpMode {

    private Servo shooterHood;
    private Servo shooterStop;

    @Override
    public void runOpMode() {
        shooterHood = hardwareMap.get(Servo.class, "shooterHood");
        shooterStop = hardwareMap.get(Servo.class, "shooterStop");

        telemetry.addData("Status", "Ready!");
        telemetry.addLine("A = 0.0 (Closed)");
        telemetry.addLine("B = 1.0 (Open)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Hood control
            if (gamepad1.a) {
                shooterHood.setPosition(0.0);
            } else if (gamepad1.b) {
                shooterHood.setPosition(1.0);
            }

            // Stop control
            if (gamepad1.x) {
                shooterStop.setPosition(0.0);
            } else if (gamepad1.y) {
                shooterStop.setPosition(1.0);
            }

            // Telemetry
            telemetry.addLine("=== SERVO TESTER ===");
            telemetry.addLine();
            telemetry.addData("Hood Position", "%.3f", shooterHood.getPosition());
            telemetry.addData("Stop Position", "%.3f", shooterStop.getPosition());
            telemetry.addLine();
            telemetry.addLine("HOOD: A=0.0 | B=1.0");
            telemetry.addLine("STOP: X=0.0 | Y=1.0");
            telemetry.update();
        }
    }
}
