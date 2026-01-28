package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

public class ShooterController {
    public Gamepad gamepad;
    private Shooter shooter;
    private Vision vision;

    private boolean prevRightBumper = false;

    public ShooterController(Gamepad gamepad, Shooter shooter,  Vision vision) {
        this.gamepad = gamepad;
        this.shooter = shooter;
        this.vision = vision;
    }

    public void update(Intake intake) {
        if (gamepad == null) return;

        // === Right Bumper - запуск стрельбы ===
        if (gamepad.right_bumper && !prevRightBumper) {
            shooter.startShoot();
        }

        // Автоматическая регулировка hood на основе расстояния от Vision
        if (vision != null && vision.hasTargetTag()) {
            double distance = vision.getTargetDistance();
            if (distance > 0) {
                shooter.updateHood(distance);
            }
        }

        // Обновление FSM шутера
        shooter.updateFSM(intake);

        // Сохраняем состояние кнопки
        prevRightBumper = gamepad.right_bumper;
    }

    public boolean isShooting() {
        return shooter.isShooting();
    }

    public Shooter.ShooterState getCurrentState() {
        return shooter.getCurrentState();
    }
}