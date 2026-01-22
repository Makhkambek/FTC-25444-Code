package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

public class ShooterController {
    public Gamepad gamepad;
    private Shooter shooter;
    private Vision vision;

    public boolean shooterRunning = true;

    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;
    private boolean prevXButton = false;

    public ShooterController(Gamepad gamepad, Shooter shooter,  Vision vision) {
        this.gamepad = gamepad;
        this.shooter = shooter;
        this.vision = vision;
    }

    public void update() {
        if (gamepad == null) return;

        // === Shooter On/Off ===
        if (gamepad.right_bumper && !prevRightBumper) {
            shooterRunning = true;
        }

        if (gamepad.left_bumper && !prevLeftBumper) {
            shooterRunning = false;
        }

        if (shooterRunning) {
            shooter.on();
        } else {
            shooter.off();
        }

        // === X Button - запуск стрельбы ===
//        if (gamepad.x && !prevXButton) {
//            startFiring();
//        }


        // Сохраняем состояния
        prevRightBumper = gamepad.right_bumper;
        prevLeftBumper = gamepad.left_bumper;
        prevXButton = gamepad.x;
    }

//    private void startFiring() {
//        // Не стреляем если уже стреляем
//        if (sorter.isBusy()) {
//            return;
//        }
//
//        // Получаем последовательность от Vision
//        Sorter.ShootSequence sequence = vision.getShootSequence();
//
//        if (sequence != null) {
//            sorter.startSmartSequence(sequence);
//        } else {
//            sorter.startDefaultSequence();
//        }
//    }

    public boolean isRunning() {
        return shooterRunning;
    }

    public void setRunning(boolean running) {
        this.shooterRunning = running;
    }

//    public boolean isFiring() {
//        return sorter.isBusy();
//    }
}