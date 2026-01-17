package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.Shooter;

public class ShooterController {
    private Gamepad gamepad;
    private Shooter shooter;

    private boolean shooterRunning = true;

    // Предыдущие состояния кнопок (для toggle)
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;

    public ShooterController(Gamepad gamepad, Shooter shooter) {
        this.gamepad = gamepad;
        this.shooter = shooter;
    }


    public void update() {
        // Toggle on
        if (gamepad.right_bumper && !prevRightBumper) {
            shooterRunning = true;
        }

        // Toggle off
        if (gamepad.left_bumper && !prevLeftBumper) {
            shooterRunning = false;
        }

        // Применяем состояние
        if (shooterRunning) {
            shooter.on();
        } else {
            shooter.off();
        }

        // Сохраняем предыдущие состояния
        prevRightBumper = gamepad.right_bumper;
        prevLeftBumper = gamepad.left_bumper;
    }

    public boolean isRunning() {
        return shooterRunning;
    }

    public void setRunning(boolean running) {
        this.shooterRunning = running;
    }
}