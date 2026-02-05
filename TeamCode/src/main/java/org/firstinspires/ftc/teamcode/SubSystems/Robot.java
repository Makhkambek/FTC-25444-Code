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

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean isRedAlliance) {
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
    }

    public void start() {
        // Начальная настройка - убедимся что intake выключен
        intake.off();

        // Запускаем shooter моторы с velocity на основе начального расстояния до цели
        double distanceToGoal = turret.getDistanceToGoal();
        if (distanceToGoal > 0) {
            shooter.updateVelocity(distanceToGoal); // Устанавливает правильную velocity
            shooter.updateHood(distanceToGoal); // Устанавливает правильную hood позицию
        } else {
            shooter.on(); // Fallback: стандартная velocity = 2000 ticks/sec
        }
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        follower.update();

        driveTrain.drive(gamepad1, gamepad2, telemetry);

        // Динамически обновляем Hood на основе расстояния до цели (Vision)
        shooter.updateHoodDynamic(vision);

        // Динамически обновляем target velocity на основе расстояния до цели (Odometry)
        double distanceToGoal = turret.getDistanceToGoal();
        if (distanceToGoal > 0) {
            shooter.updateVelocity(distanceToGoal);
            shooter.updateHood(distanceToGoal); // Также обновляем Hood
        }

        shooter.updatePID();

        // Автоматическая регулировка Hood и Turret происходит внутри контроллеров
        updateControllers(gamepad2);

        // Fire button
        handleFireButton(gamepad2, telemetry);
    }

private void updateControllers(Gamepad gamepad2) {
        shooterController.gamepad = gamepad2;
        shooterController.update(intake);

        turretController.gamepad = gamepad2;
        turretController.update();

        // IntakeController управляет intake только если Shooter НЕ активен
        intakeController.gamepad = gamepad2;
        if (!shooterController.isShooting()) {
            intakeController.update();
        }

        resetController.handleResetButton(gamepad2);
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
