package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class ResetController {
    private HeadingController headingController;
    private IntakeController intakeController;
    private ShooterController shooterController;
    private TurretController turretController;

    private Intake intake;
    private Shooter shooter;
    private Turret turret;

    private boolean wasResetPressed = false;

    public ResetController(HeadingController headingController,
                           IntakeController intakeController,
                           ShooterController shooterController,
                           TurretController turretController,
                           Intake intake,
                           Shooter shooter,
                           Turret turret) {
        this.headingController = headingController;
        this.intakeController = intakeController;
        this.shooterController = shooterController;
        this.turretController = turretController;
        this.intake = intake;
        this.shooter = shooter;
        this.turret = turret;
    }

    public void handleResetButton(Gamepad gamepad2) {
        if (gamepad2.options && !wasResetPressed) {
            wasResetPressed = true;
            resetControls();
        }
        if (!gamepad2.options) wasResetPressed = false;
    }

    private void resetControls() {
        // Сброс контроллеров
        turretController.enableAutoAim();

        // Сброс heading контроллера
        headingController.reset();

        // Сброс подсистем
        intake.off();
        shooter.reset(); // Полный сброс shooter (моторы, servos, FSM)

        // Снова запускаем моторы с правильной velocity на основе расстояния
        double distanceToGoal = turret.getDistanceToGoal();
        if (distanceToGoal > 0) {
            shooter.updateVelocity(distanceToGoal);
            shooter.updateHood(distanceToGoal);
        } else {
            shooter.on(); // Fallback: стандартная velocity
        }

        turret.returnToCenter(); // Возврат turret на 0 с PID

        wasResetPressed = false;
    }

    public boolean isResetting() {
        // Turret больше не имеет состояния "resetting"
        // Просто проверяем, находится ли он в центре
        return !turret.isCentered();
    }
}