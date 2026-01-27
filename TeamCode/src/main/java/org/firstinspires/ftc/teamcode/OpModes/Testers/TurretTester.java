package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;
// import org.firstinspires.ftc.teamcode.SubSystems.Vision;

@Config
@TeleOp(name="[TEST] Turret Tester", group="Testers")
public class TurretTester extends LinearOpMode {

    // PID коэффициенты для настройки через Dashboard
    public static double KP = 0.03;
    public static double KI = 0.0;
    public static double KD = 0.01;

    private Turret turret;
    // private Vision vision;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;

    private enum TestMode {
        MANUAL,     // Ручное управление с PID
        AUTO        // Auto-aim / scanning (с Vision)
    }

    private TestMode currentMode = TestMode.MANUAL;
    private double targetAngle = 0.0;

    @Override
    public void runOpMode() {
        // Настройка телеметрии для Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // vision = new Vision();
        // vision.init(hardwareMap);
        // vision.start();
        // vision.setAlliance(true); // RED

        turret = new Turret(hardwareMap, null); // Pass null for vision in testing

        waitForStart();

        // Reset encoder
        turret.resetEncoder();

        while (opModeIsActive()) {
            // Обновляем PID коэффициенты из Dashboard
            turret.setPID(KP, KI, KD);

            handleControls();
        }

        turret.stop();
        // vision.stop();
    }

    private void handleControls() {
        if (currentMode == TestMode.MANUAL) {
            // === MANUAL MODE ===

            // Right stick for smooth manual control with PID
            double stickInput = -gamepad1.right_stick_x;
            if (Math.abs(stickInput) > 0.1) {
                turret.syncManualTarget();
                turret.manualControl(stickInput);
            }

            // Preset positions
            if (gamepad1.dpad_left && !prevA) {
                targetAngle = -90.0; // Max left
            }
            if (gamepad1.dpad_up && !prevB) {
                targetAngle = 0.0; // Center
            }
            if (gamepad1.dpad_right && !prevX) {
                targetAngle = 90.0; // Max right
            }

            // Apply target if set by dpad
            if (gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right) {
                turret.syncManualTarget();
                // Move towards target using manual control
            }

            // Center command
            if (gamepad1.a && !prevA) {
                turret.returnToCenter();
            }

            // Stop
            if (gamepad1.b && !prevB) {
                turret.stop();
            }

        } else {
            // === AUTO MODE ===
            // Auto-aim with vision (uncomment when ready)
            // turret.autoAim();

            // For now, just show what would happen
            telemetry.addLine("AUTO MODE - Vision integration needed");
            telemetry.addLine("Uncomment vision code to enable");
        }

        // Mode switching
        if (gamepad1.y && !prevY) {
            currentMode = (currentMode == TestMode.MANUAL) ? TestMode.AUTO : TestMode.MANUAL;
            if (currentMode == TestMode.AUTO) {
                // Prepare for auto mode
                turret.syncManualTarget();
            }
        }

        // Save button states
        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevY = gamepad1.y;
    }

}
