package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

public class TurretController {
    public Gamepad gamepad;
    private Turret turret;
    private Vision vision;

    public boolean autoAimEnabled = true;

    // Чувствительность ручного управления
    private static final double MANUAL_SENSITIVITY = 0.3;

    // Deadzone для джойстика
    private static final double JOYSTICK_DEADZONE = 0.1;

    public TurretController(Gamepad gamepad, Turret turret, Vision vision) {
        this.gamepad = gamepad;
        this.turret = turret;
        this.vision = vision;
    }

    public void update() {
        if (gamepad == null) return;

        double manualInput = gamepad.right_stick_x;

        if (Math.abs(manualInput) > JOYSTICK_DEADZONE) {
            // Ручное управление - отменяем autoAim
            if (autoAimEnabled) {
                // Переключаемся с auto на manual - синхронизируем target
                turret.syncManualTarget();
                autoAimEnabled = false;
            }
            turret.manualControl(manualInput * MANUAL_SENSITIVITY);
        } else {
            // Auto-aim
            if (!autoAimEnabled) {
                // Переключаемся с manual на auto
                autoAimEnabled = true;
            }
            turret.autoAim();
        }
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    public void enableAutoAim() {
        autoAimEnabled = true;
    }

    public void disableAutoAim() {
        autoAimEnabled = false;
    }
}