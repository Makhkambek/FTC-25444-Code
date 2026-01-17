package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import Controllers.HeadingController;
import Controllers.IntakeController;
import Controllers.ShooterController;
import Controllers.TurretController;
import Controllers.ResetController;

public class Robot {
    // SubSystems
    public Localizer localizer;
    public DriveTrain driveTrain;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Sorter sorter;
    public Vision vision;

    // Controllers
    public HeadingController headingController;
    public IntakeController intakeController;
    public ShooterController shooterController;
    public TurretController turretController;
    public ResetController resetController;

    // Fire button state
    private boolean prevFireButton = false;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        // Localizer первым!
        localizer = Localizer.getInstance(hardwareMap);

        // SubSystems
        driveTrain = new DriveTrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        sorter = new Sorter(hardwareMap);
        vision = new Vision();

        vision.init(hardwareMap);
        vision.start();

        // HeadingController (используется в DriveTrain)
        headingController = new HeadingController(hardwareMap);

        // Controllers (на gamepad2)
        intakeController = new IntakeController(null, intake); // gamepad передадим в update
        shooterController = new ShooterController(null, shooter);
        turretController = new TurretController(null, turret);
        resetController = new ResetController(null, intake, shooter, turret);
    }

    public void start() {
        // Начальная настройка
        sorter.scanBalls();
        shooter.autoAdjustHood(vision);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        // Обновляем Localizer (один раз!)
        localizer.update();

        // Sorter постоянно сканирует
        sorter.scanBalls();

        // Shooter автонастройка Hood
        shooter.autoAdjustHood(vision);

        // DriveTrain
        driveTrain.drive(gamepad1, gamepad2, telemetry);

        // Контроллеры (передаем gamepad2)
        updateControllers(gamepad2);

        // Fire button
        handleFireButton(gamepad2, telemetry);
    }

    private void updateControllers(Gamepad gamepad2) {
        intakeController.gamepad = gamepad2;
        intakeController.update();

        shooterController.gamepad = gamepad2;
        shooterController.update();

        turretController.gamepad = gamepad2;
        turretController.update();

        resetController.gamepad = gamepad2;
        resetController.update();
    }

    private void handleFireButton(Gamepad gamepad2, Telemetry telemetry) {
        if (gamepad2.a && !prevFireButton) {
            fireBalls(telemetry);
        }
        prevFireButton = gamepad2.a;
    }

    private void fireBalls(Telemetry telemetry) {
        Sorter.ShootSequence sequence = vision.getShootSequence();

        if (sequence != null && sorter.isValidConfiguration()) {
            sorter.executeSmartSequence(sequence);
            telemetry.addData("🔥 FIRING", sequence.toString());
        } else {
            sorter.executeDefaultSequence();
            telemetry.addData("🔥 FIRING", "Default");
        }
    }

    public void stop() {
        intake.off();
        shooter.off();
        turret.stop();
        vision.stop();
    }
}