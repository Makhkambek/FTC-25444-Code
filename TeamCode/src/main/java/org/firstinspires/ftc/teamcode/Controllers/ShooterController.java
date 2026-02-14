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

        // === Dpad Up - ручное открытие shooterStop (приоритет над FSM) ===
        if (gamepad.dpad_up) {
            // Зажата - держим shooterStop открытым, FSM не может закрыть
            shooter.setManualStopOverride(true);
        } else if (gamepad.dpad_down) {
            // Dpad Down - ручное закрытие shooterStop
            shooter.forceCloseStop();
        } else {
            // Отпущена - FSM работает как обычно
            shooter.setManualStopOverride(false);
        }

        // === Right Bumper - запуск стрельбы ===
        if (gamepad.right_bumper && !prevRightBumper) {
            shooter.startShoot();
        }

        // Hood и Velocity обновляются от Vision (приоритет) с fallback на Odometry в Robot.update()

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