package org.firstinspires.ftc.teamcode.Controllers;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ResetController {
    public Gamepad gamepad;

    private HeadingController headingController;
    private IntakeController intakeController;
    private ShooterController shooterController;
    private TurretController turretController;

    private boolean prevOptionsPressed = false;

    public ResetController(Gamepad gamepad, HeadingController headingController,
                           IntakeController intakeController, ShooterController shooterController,
                           TurretController turretController) {
        this.gamepad = gamepad;
        this.headingController = headingController;
        this.intakeController = intakeController;
        this.shooterController = shooterController;
        this.turretController = turretController;
    }

    public void update() {
        if (gamepad == null) return;

        if (gamepad.options && !prevOptionsPressed) {
            resetAll();
        }
        prevOptionsPressed = gamepad.options;
    }

    private void resetAll() {
        if (headingController != null) headingController.reset();
        if (intakeController != null) intakeController.reset();
        if (shooterController != null) shooterController.reset();
        if (turretController != null) turretController.reset();
    }
}