package org.firstinspires.ftc.teamcode.Controller;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.VisionSubsystem;

public class ShootingController {

    private enum ShootingState {
        MANUAL,
        TARGET_MODE_SEARCHING,
        TARGET_MODE_AUTO_AIM,
        READY_TO_SHOOT,
        SHOOTING
    }

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final VisionSubsystem vision;
    private final Telemetry telemetry;

    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    private ShootingState currentState;

    private boolean previousRightTrigger;
    private boolean previousLeftTrigger;

    private static final double YAW_LOCK_THRESHOLD = 3.0;
    private static final double TICKS_PER_DEGREE = 1.2;
    private static final double JOYSTICK_DEADZONE = 0.1;

    private static final double DISTANCE_STEP_CM = 15.0;
    private static final double HOOD_INCREMENT = 0.03;
    private static final double VELOCITY_BASE = 800.0;
    private static final double VELOCITY_INCREMENT = 21.0;

    private ElapsedTime shootingTimer;
    private static final double SHOOTING_DURATION_SECONDS = 5.0;

    private double targetHoodPosition;
    private double targetShooterVelocity;

    private static final int RUMBLE_DURATION_MS = 500;

    public ShootingController(
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            VisionSubsystem vision,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Telemetry telemetry
    ) {
        this.shooter = shooter;
        this.intake = intake;
        this.vision = vision;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        this.currentState = ShootingState.MANUAL;
        this.previousRightTrigger = false;
        this.previousLeftTrigger = false;
        this.shootingTimer = new ElapsedTime();

        this.targetHoodPosition = 0.0;
        this.targetShooterVelocity = ShooterSubsystem.DEFAULT_VELOCITY;
    }

    public void update() {
        vision.update();

        boolean rightTriggerPressed = detectEdge(gamepad2.right_trigger > 0.1, previousRightTrigger);
        boolean leftTriggerPressed = detectEdge(gamepad2.left_trigger > 0.1, previousLeftTrigger);

        previousRightTrigger = gamepad2.right_trigger > 0.1;
        previousLeftTrigger = gamepad2.left_trigger > 0.1;

        switch (currentState) {
            case MANUAL:
                handleManualState(rightTriggerPressed);
                break;

            case TARGET_MODE_SEARCHING:
                handleSearchingState();
                break;

            case TARGET_MODE_AUTO_AIM:
                handleAutoAimState();
                break;

            case READY_TO_SHOOT:
                handleReadyState(leftTriggerPressed);
                break;

            case SHOOTING:
                handleShootingState();
                break;
        }

        updateTelemetry();
    }

    private void handleManualState(boolean rightTriggerPressed) {
        double turretPower = -gamepad2.right_stick_x;
        if (Math.abs(turretPower) > JOYSTICK_DEADZONE) {
            shooter.setTurretManualPower(turretPower * 0.5);
        } else {
            shooter.setTurretManualPower(0.0);
        }

        if (rightTriggerPressed) {
            currentState = ShootingState.TARGET_MODE_SEARCHING;
            telemetry.addLine("Entering target mode - search for tag");
        }
    }

    private void handleSearchingState() {
        double turretPower = -gamepad2.right_stick_x;
        if (Math.abs(turretPower) > JOYSTICK_DEADZONE) {
            shooter.setTurretManualPower(turretPower * 0.5);
        } else {
            shooter.setTurretManualPower(0.0);
        }

        if (vision.hasTarget()) {
            calculateTargetingParameters();

            shooter.setHoodPosition(targetHoodPosition);
            shooter.setShooterVelocity(targetShooterVelocity);

            currentState = ShootingState.TARGET_MODE_AUTO_AIM;
            telemetry.addLine("Target acquired - auto-aiming");
        }
    }

    private void handleAutoAimState() {
        if (!vision.hasTarget()) {
            currentState = ShootingState.TARGET_MODE_SEARCHING;
            return;
        }

        double yawDegrees = vision.getYawDegrees();

        int deltaTicks = (int) Math.round(yawDegrees * TICKS_PER_DEGREE);
        int currentTicks = shooter.getTurretCurrentTicks();
        int targetTicks = currentTicks + deltaTicks;

        shooter.setTurretTargetTicks(targetTicks);

        if (Math.abs(yawDegrees) < YAW_LOCK_THRESHOLD) {
            currentState = ShootingState.READY_TO_SHOOT;

            rumbleGamepads();

            telemetry.addLine("LOCKED ON - Press left trigger to shoot");
        }
    }

    private void handleReadyState(boolean leftTriggerPressed) {
        shooter.holdTurretPosition();

        if (leftTriggerPressed) {
            currentState = ShootingState.SHOOTING;
            shootingTimer.reset();

            intake.intakeIn();

            telemetry.addLine("SHOOTING - 5 second sequence started");
        }
    }

    private void handleShootingState() {
        if (shootingTimer.seconds() < SHOOTING_DURATION_SECONDS) {

        } else {
            intake.stop();

            shooter.setShooterDefaultVelocity();

            currentState = ShootingState.MANUAL;

            telemetry.addLine("Shooting complete - returning to manual mode");
        }
    }

    private void calculateTargetingParameters() {
        double distanceCm = vision.getDistanceCm();

        int stepIndex = Math.max(1, (int) Math.ceil(distanceCm / DISTANCE_STEP_CM));

        targetHoodPosition = HOOD_INCREMENT * stepIndex;
        targetHoodPosition = Math.min(targetHoodPosition, ShooterSubsystem.HOOD_MAX);

        targetShooterVelocity = VELOCITY_BASE + (VELOCITY_INCREMENT * stepIndex);

        telemetry.addData("Distance", "%.1f cm", distanceCm);
        telemetry.addData("Step Index", stepIndex);
        telemetry.addData("Target Hood", "%.2f", targetHoodPosition);
        telemetry.addData("Target Velocity", "%.0f tps", targetShooterVelocity);
    }

    private void rumbleGamepads() {
        gamepad1.rumble(RUMBLE_DURATION_MS);
        gamepad2.rumble(RUMBLE_DURATION_MS);
    }

    private boolean detectEdge(boolean current, boolean previous) {
        return current && !previous;
    }

    private void updateTelemetry() {
        telemetry.addData("Shooting State", currentState);
        telemetry.addData("Has Target", vision.hasTarget());

        if (vision.hasTarget()) {
            telemetry.addData("Yaw", "%.2f deg", vision.getYawDegrees());
            telemetry.addData("Distance", "%.1f cm", vision.getDistanceCm());
        }

        if (currentState == ShootingState.SHOOTING) {
            telemetry.addData("Shooting Time", "%.1f / %.1f sec",
                    shootingTimer.seconds(), SHOOTING_DURATION_SECONDS);
        }
    }
}