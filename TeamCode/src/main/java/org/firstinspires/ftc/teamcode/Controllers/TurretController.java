package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class TurretController {
    private Gamepad gamepad;
    private Turret turret;

    public boolean autoAimEnabled = true;

    // Чувствительность ручного управления
    private static final double MANUAL_SENSITIVITY = 0.3;

    // Deadzone для джойстика
    private static final double JOYSTICK_DEADZONE = 0.1;

    public TurretController(Gamepad gamepad, Turret turret) {
        this.gamepad = gamepad;
        this.turret = turret;
    }

    public void update() {
        double manualInput = gamepad.right_stick_x;

        if (Math.abs(manualInput) > JOYSTICK_DEADZONE) {
            // Ручное управление
            autoAimEnabled = false;
            turret.manualControl(manualInput * MANUAL_SENSITIVITY);
        } else {
            // Auto-aim
            autoAimEnabled = true;
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