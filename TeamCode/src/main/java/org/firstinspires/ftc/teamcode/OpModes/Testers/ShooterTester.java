package org.firstinspires.ftc.teamcode.OpModes.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
// import org.firstinspires.ftc.teamcode.SubSystems.Vision;

@Config
@TeleOp(name="[TEST] Shooter Tester", group="Testers")
public class ShooterTester extends LinearOpMode {

    // PID коэффициенты для настройки через Dashboard
    public static double KP = 0.005;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KF = 0.0;
    public static double TARGET_VELOCITY = 2200; // ticks/sec (78% от max 2800)

    private Shooter shooter;
    private Intake intake;
    // private Vision vision;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    private enum TestMode {
        BASIC,      // Основной тест: hood servo, моторы, stop servo
        ADVANCED    // Продвинутый: FSM + Vision integration
    }

    private TestMode currentMode = TestMode.BASIC;
    private boolean motorsRunning = false;

    @Override
    public void runOpMode() {
        // Настройка телеметрии для Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initializing Shooter...");
        telemetry.update();

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        // vision = new Vision();
        // vision.init(hardwareMap);
        // vision.start();
        // vision.setAlliance(true); // RED

        telemetry.addData("Status", "Ready!");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Обновляем PID коэффициенты из Dashboard
            shooter.kP = KP;
            shooter.kI = KI;
            shooter.kD = KD;
            shooter.kF = KF;

            // Обновляем PID для моторов
            shooter.updatePID();

            handleControls();
            displayTelemetry();
            telemetry.update();
        }

        shooter.reset();
        intake.off();
        // vision.stop();
    }

    private void handleControls() {
        // === BASIC MODE Controls ===
        if (currentMode == TestMode.BASIC) {
            // Hood Position Control
            if (gamepad1.dpad_up && !prevDpadUp) {
                shooter.setHoodPosition(Shooter.HoodPosition.FAR);
            }
            if (gamepad1.dpad_left && !prevDpadLeft) {
                shooter.setHoodPosition(Shooter.HoodPosition.MIDDLE);
            }
            if (gamepad1.dpad_down && !prevDpadDown) {
                shooter.setHoodPosition(Shooter.HoodPosition.CLOSE);
            }

            // Shooter Motors Control
            if (gamepad1.a && !prevA) {
                motorsRunning = !motorsRunning;
                if (motorsRunning) {
                    shooter.setTargetVelocity(TARGET_VELOCITY);
                } else {
                    shooter.off();
                }
            }

            // Shooter Stop Servo Control
            if (gamepad1.right_bumper && !prevRightBumper) {
                shooter.openStop(); // Open stop servo (позволяет ball пройти)
            }
            if (gamepad1.left_bumper && !prevLeftBumper) {
                shooter.closeStop(); // Close stop servo
            }

            // Reset
            if (gamepad1.b && !prevB) {
                shooter.reset();
                motorsRunning = false;
            }
        }

        // === ADVANCED MODE Controls ===
        if (currentMode == TestMode.ADVANCED) {
            // FSM Test
            if (gamepad1.a && !prevA) {
                shooter.startShoot();
            }

            // Update FSM
            shooter.updateFSM(intake);

            // Vision-based hood adjustment (uncomment when vision is ready)
            // if (vision.hasTargetTag()) {
            //     double distance = vision.getTargetDistance();
            //     if (distance > 0) {
            //         shooter.updateHood(distance);
            //     }
            // }
        }

        // Mode switching
        if (gamepad1.y && !prevY) {
            currentMode = (currentMode == TestMode.BASIC) ? TestMode.ADVANCED : TestMode.BASIC;
            shooter.reset();
            motorsRunning = false;
        }

        // Save button states
        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevY = gamepad1.y;
        prevRightBumper = gamepad1.right_bumper;
        prevLeftBumper = gamepad1.left_bumper;
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
    }

    private void displayTelemetry() {
        telemetry.addLine("=== SHOOTER TESTER ===");
        telemetry.addData("Mode", currentMode);
        telemetry.addLine();

        // Status
        telemetry.addLine("--- STATUS ---");
        telemetry.addData("Hood Position", shooter.getCurrentHoodPosition());
        telemetry.addData("Stop Position", "%.3f", shooter.getStopPosition());
        telemetry.addData("Stop Status", shooter.getStopPosition() > 0.5 ? "OPEN" : "CLOSED");
        telemetry.addData("Motors Running", shooter.isRunning() ? "YES" : "NO");
        telemetry.addData("Shooter State", shooter.getCurrentState());
        telemetry.addData("Is Shooting", shooter.isShooting() ? "YES" : "NO");

        telemetry.addLine();

        // PID Status
        telemetry.addLine("--- PID ---");
        telemetry.addData("Target Velocity", "%.0f ticks/sec", shooter.getTargetVelocity());
        telemetry.addData("Current Velocity", "%.0f ticks/sec", shooter.getCurrentVelocity());
        double error = shooter.getTargetVelocity() - shooter.getCurrentVelocity();
        telemetry.addData("Error", "%.0f ticks/sec", error);
        telemetry.addData("kP", "%.4f", shooter.kP);
        telemetry.addData("kI", "%.4f", shooter.kI);
        telemetry.addData("kD", "%.4f", shooter.kD);
        telemetry.addData("kF", "%.4f", shooter.kF);

        telemetry.addLine();

        if (currentMode == TestMode.BASIC) {
            telemetry.addLine("=== BASIC MODE CONTROLS ===");
            telemetry.addData("DPad Up", "Hood FAR (1.0)");
            telemetry.addData("DPad Left", "Hood MIDDLE (0.5)");
            telemetry.addData("DPad Down", "Hood CLOSE (0.0)");
            telemetry.addData("Right Bumper", "OPEN Stop Servo");
            telemetry.addData("Left Bumper", "CLOSE Stop Servo");
            telemetry.addData("A", "Toggle Motors ON/OFF");
            telemetry.addData("B", "RESET ALL");
            telemetry.addData("Y", "Switch to ADVANCED Mode");
        } else {
            telemetry.addLine("=== ADVANCED MODE CONTROLS ===");
            telemetry.addData("A", "Start Shoot Sequence");
            telemetry.addData("B", "RESET ALL");
            telemetry.addData("Y", "Switch to BASIC Mode");
            telemetry.addLine();
            telemetry.addLine("FSM auto-updates each loop");

            // Vision info (uncomment when ready)
            // telemetry.addLine();
            // telemetry.addLine("--- VISION ---");
            // telemetry.addData("Target Visible", vision.hasTargetTag() ? "YES" : "NO");
            // if (vision.hasTargetTag()) {
            //     telemetry.addData("Distance (cm)", "%.1f", vision.getTargetDistance());
            // }
        }
    }
}
