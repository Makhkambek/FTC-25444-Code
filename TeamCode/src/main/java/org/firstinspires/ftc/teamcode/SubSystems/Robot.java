package org.firstinspires.ftc.teamcode.SubSystems;
import org.firstinspires.ftc.teamcode.SubSystems.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Controllers.HeadingController;
import org.firstinspires.ftc.teamcode.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.Controllers.ShooterController;
import org.firstinspires.ftc.teamcode.Controllers.TurretController;
import org.firstinspires.ftc.teamcode.Controllers.ResetController;

public class
Robot {
    // SubSystems
    public Localizer localizer;
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
        // Localizer первым!
        localizer = Localizer.getInstance(hardwareMap);

        // Vision (нужна для Turret)
        vision = new Vision();
        vision.init(hardwareMap);
        vision.start();
        vision.setAlliance(isRedAlliance); // Устанавливаем альянс

        // SubSystems
        driveTrain = new DriveTrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, vision);

        // HeadingController (используется в DriveTrain)
        headingController = new HeadingController(hardwareMap);

        // Controllers (на gamepad2)
        intakeController = new IntakeController(null, intake); // gamepad передадим в update
        shooterController = new ShooterController(null, shooter, vision);
        turretController = new TurretController(null, turret, vision);
        resetController = new ResetController(headingController, intakeController, shooterController, turretController, intake, shooter, turret);
    }

    public void start() {
        // Начальная настройка
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        localizer.update();

        driveTrain.drive(gamepad1, gamepad2, telemetry);

        // Динамически обновляем Hood на основе расстояния до цели
        // Приоритет: Vision -> Odometry
        shooter.updateHoodDynamic(turret, vision);

        shooter.updatePID();

        // Автоматическая регулировка Hood и Turret происходит внутри контроллеров
        updateControllers(gamepad2);

        // Fire button
        handleFireButton(gamepad2, telemetry);
    }

    private void updateControllers(Gamepad gamepad2) {
        intakeController.update();

        shooterController.gamepad = gamepad2;
        shooterController.update(intake);

        turretController.gamepad = gamepad2;
        turretController.update();

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
