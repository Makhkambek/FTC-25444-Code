package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;

@TeleOp(name="[TEST] Servo Tester", group="Testers")
public class ServoTester extends LinearOpMode {

    private Servo shooterHood;
    private Servo shooterStop;
    private Intake intake;

    @Override
    public void runOpMode() {
//        shooterHood = hardwareMap.get(Servo.class, "shooterHood");
        shooterStop = hardwareMap.get(Servo.class, "shooterStop");
        intake = new Intake(hardwareMap);

        telemetry.addData("Status", "Ready!");
        telemetry.addLine("A = 0.0 (Closed)");
        telemetry.addLine("B = 1.0 (Open)");
        telemetry.addLine("Right Trigger = Intake ON");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Hood control
//            if (gamepad1.a) {
//                shooterHood.setPosition(0.0);
//            } else if (gamepad1.b) {
//                shooterHood.setPosition(0.5);
//            }

            // Stop control
            if (gamepad1.dpad_down) {
                shooterStop.setPosition(0.0);
            } else if (gamepad1.dpad_up) {
                shooterStop.setPosition(0.29);
            }

            // Intake control
            if (gamepad1.right_trigger > 0.1) {
                intake.on();
            } else {
                intake.off();
            }

            // Telemetry
            telemetry.addLine("=== SERVO TESTER ===");
            telemetry.addLine();
//            telemetry.addData("Hood Position", "%.3f", shooterHood.getPosition());
            telemetry.addData("Stop Position", "%.3f", shooterStop.getPosition());
            telemetry.addData("Intake", gamepad1.right_trigger > 0.1 ? "ON" : "OFF");
            telemetry.addLine();
            telemetry.addLine("HOOD: A=0.0 | B=1.0");
            telemetry.addLine("STOP: X=0.0 | Y=0.29");
            telemetry.addLine("INTAKE: Right Trigger");
            telemetry.update();
        }

        // Cleanup
        intake.off();
    }
}
