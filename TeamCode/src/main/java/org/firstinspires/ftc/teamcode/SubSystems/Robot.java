package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.Controllers.HeadingController;
import org.firstinspires.ftc.teamcode.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.Controllers.ShooterController;
import org.firstinspires.ftc.teamcode.Controllers.TurretController;
import org.firstinspires.ftc.teamcode.Controllers.ResetController;
import org.firstinspires.ftc.teamcode.OpModes.TeleOpMode;

public class
Robot {
    // SubSystems
    public Follower follower;
    public DriveTrain driveTrain;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Vision vision;

    // Controllers
    public HeadingController headingController;
    public IntakeController intakeController;
    public ShooterController shooterController;
    public TurretController turretController;
    public ResetController resetController;

    // Fire button state
    private boolean prevFireButton = false;

    // Distance source для debug телеметрии (Vision или Odometry)
    public String distanceSource = "N/A";

    // TeleOp mode (NORMAL or EMERGENCY)
    private TeleOpMode teleOpMode;

    // Manual hood control flag
    public boolean manualHoodMode = false;

    // Hood jitter prevention
    private double lastHoodDistance = -1.0;
    private static final double HOOD_UPDATE_THRESHOLD_CM = 3.0; // Only update hood if distance changes by 3cm

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean isRedAlliance, TeleOpMode mode) {
        this.teleOpMode = mode;
        // Pedro Pathing Follower (одометрия)
        follower = Constants.createFollower(hardwareMap);
        follower.update(); // CRITICAL: Initialize before setting pose

        // Vision (нужна для Turret) - ENABLED for Kalman Filter
        vision = new Vision();
        vision.init(hardwareMap);
        vision.start();
        vision.setAlliance(isRedAlliance); // Устанавливаем альянс

        // SubSystems
        driveTrain = new DriveTrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, vision, follower); // Vision enabled for Kalman Filter

        // HeadingController (используется в DriveTrain)
        headingController = new HeadingController(hardwareMap);

        // Controllers (на gamepad2)
        intakeController = new IntakeController(null, intake); // gamepad передадим в update
        shooterController = new ShooterController(null, shooter, vision); // Vision enabled
        turretController = new TurretController(null, turret, vision); // Vision enabled
        resetController = new ResetController(headingController, intakeController, shooterController, turretController, intake, shooter, turret);

        // Set initial auto-aim state based on mode
        if (mode == TeleOpMode.EMERGENCY) {
            turretController.disableAutoAim();
        }
        // NORMAL mode keeps default autoAimEnabled=true
    }

    public void start() {
        // Начальная настройка - убедимся что intake выключен
        intake.off();

        // Only auto-aim on startup in NORMAL mode
        if (teleOpMode == TeleOpMode.NORMAL) {
            // Турель сразу смотрит на goal при старте
            turret.autoAim();

            // Запускаем shooter моторы с velocity на основе начального расстояния до цели
            double distanceToGoal = turret.getDistanceToGoal();
            if (distanceToGoal > 0) {
                shooter.updateVelocity(distanceToGoal); // Устанавливает правильную velocity
                shooter.updateHood(distanceToGoal); // Устанавливает правильную hood позицию
            } else {
                shooter.on(); // Fallback: стандартная velocity = 2000 ticks/sec
            }
        } else {
            // EMERGENCY mode: just turn on shooter with fallback settings
            shooter.on();
            // Turret stays at current encoder position (0° if just powered on)
        }
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        follower.update();

        driveTrain.drive(gamepad1, gamepad2, telemetry);

        // Динамически обновляем velocity и hood на основе расстояния до цели
        // ТОЛЬКО Vision - если не видит тег, используем fallback (но НЕ во время стрельбы!)
        // SKIP hood updates if in manual mode
        double distanceToGoal = vision.getTargetDistance();

        if (!manualHoodMode) {
            // Auto mode - update hood based on distance
            if (distanceToGoal < 0) {
                // Камера НЕ видит tag
                if (!shooter.isShooting()) {
                    // Fallback ТОЛЬКО если НЕ стреляем (иначе мяч блокирует камеру и сбивает настройки!)
                    shooter.setTargetVelocity(1200.0);
                    shooter.setHoodPosition(0.0);  // Hood на 0
                    distanceSource = "No tag (fallback)";
                } else {
                    // Стреляем - держим последние значения velocity/hood, не меняем
                    distanceSource = "Shooting (hold last)";
                }
            } else {
                // Камера видит tag - обновляем velocity всегда, hood только если изменение > 3см
                shooter.updateVelocity(distanceToGoal);

                // Hood jitter prevention - only update if distance changed significantly
                double distanceCm = distanceToGoal * 2.54; // Convert inches to cm
                if (lastHoodDistance < 0 || Math.abs(distanceCm - lastHoodDistance) >= HOOD_UPDATE_THRESHOLD_CM) {
                    shooter.updateHood(distanceToGoal);
                    lastHoodDistance = distanceCm;
                    distanceSource = "Vision (hood updated)";
                } else {
                    distanceSource = "Vision (hood held)";
                }
            }
        } else {
            // Manual hood mode - only update velocity, not hood
            if (distanceToGoal > 0) {
                shooter.updateVelocity(distanceToGoal);
                distanceSource = "Vision (manual hood)";
            } else {
                if (!shooter.isShooting()) {
                    shooter.setTargetVelocity(1200.0);
                    distanceSource = "No tag (manual hood)";
                } else {
                    distanceSource = "Shooting (manual hood)";
                }
            }
        }

        shooter.updatePID();

        // Автоматическая регулировка Hood и Turret происходит внутри контроллеров
        updateControllers(gamepad1, gamepad2);

        // Fire button
        handleFireButton(gamepad2, telemetry);
    }

private void updateControllers(Gamepad gamepad1, Gamepad gamepad2) {
        shooterController.gamepad = gamepad2;
        shooterController.update(intake);

        turretController.gamepad = gamepad2;
        turretController.gamepad1 = gamepad1; // Для dpad calibration
        turretController.update();

        // IntakeController управляет intake только если Shooter НЕ активен
        intakeController.gamepad = gamepad2;
        if (!shooterController.isShooting()) {
            intakeController.update();
        }

        resetController.handleResetButton(gamepad2);

        // Reset manual hood mode when Options button pressed
        if (gamepad2.options) {
            manualHoodMode = false;
        }
    }

    private void handleFireButton(Gamepad gamepad2, Telemetry telemetry) {
        if (gamepad2.a && !prevFireButton) {
//            fireBalls(telemetry);
        }
        prevFireButton = gamepad2.a;
    }



    public void stop() {
        intake.off();
        shooter.off();
        turret.stop();
        vision.stop();
    }

    public void setAlliance(boolean isRedAlliance) {
        vision.setAlliance(isRedAlliance);
    }
}
