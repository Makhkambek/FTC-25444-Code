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
        shooterController.shooterRunning = false;
        turretController.autoAimEnabled = true;

        headingController.reset();
        intake.off();
        shooter.off();  //добавить серво (щас работает ток моторы)
        turret.stop(); //добавить не стоп а возвращение на 0 с PID
//        turret.resetEncoder();

        wasResetPressed = false;
    }
}