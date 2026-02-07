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
    private static final double MANUAL_SENSITIVITY = 0.7;

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

        // Left bumper - явное включение auto-aim
        if (gamepad.left_bumper) {
            if (!autoAimEnabled) {
                autoAimEnabled = true;
            }
        }

        if (Math.abs(manualInput) > JOYSTICK_DEADZONE) {
            // Manual input активен - переключаемся на manual mode
            if (autoAimEnabled) {
                turret.syncManualTarget();
                autoAimEnabled = false;
            }

            // Rumble feedback когда target visible (БЕЗ vision correction в manual!)
            if (vision != null && vision.hasTargetTag()) {
                gamepad.rumble(200);
            }

            // Только manual control БЕЗ vision correction
            turret.manualControl(manualInput * MANUAL_SENSITIVITY);

        } else {
            // Джойстик idle
            if (autoAimEnabled) {
                // Auto-aim mode - автонаведение работает
                turret.autoAim();
            } else {
                // Manual mode - держим позицию БЕЗ vision correction
                turret.manualControl(0.0);
            }
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