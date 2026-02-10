package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

public class TurretController {
    public Gamepad gamepad;
    public Gamepad gamepad1; // Для калибровки (dpad left/right)
    private Turret turret;
    private Vision vision;

    public boolean autoAimEnabled = true;

    // Чувствительность ручного управления
    private static final double MANUAL_SENSITIVITY = 0.7;

    // Deadzone для джойстика
    private static final double JOYSTICK_DEADZONE = 0.1;

    // Ручная калибровка (dpad left/right на gamepad1)
    private static final double CALIBRATION_POWER = 0.3;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    public TurretController(Gamepad gamepad, Turret turret, Vision vision) {
        this.gamepad = gamepad;
        this.turret = turret;
        this.vision = vision;
    }

    public void update() {
        if (gamepad == null) return;

        // DPAD CALIBRATION MODE - приоритет над всеми другими режимами
        // Dpad Left (gamepad1) - вращение влево без энкодеров
        // Dpad Right (gamepad1) - вращение вправо без энкодеров
        // При отпускании - сброс энкодера (текущая позиция = 0°)

        if (gamepad1 != null && gamepad1.dpad_left) {
            // Вращение влево
            turret.manualRotateRaw(-CALIBRATION_POWER);
            autoAimEnabled = false; // Отключаем auto-aim в режиме калибровки
            prevDpadLeft = true;
            return; // Пропускаем остальную логику
        } else if (gamepad1 != null && prevDpadLeft && !gamepad1.dpad_left) {
            // Dpad left отпущен - останавливаем и сбрасываем энкодер
            turret.manualRotateRaw(0.0);
            turret.resetEncoder();
            prevDpadLeft = false;
            return;
        }

        if (gamepad1 != null && gamepad1.dpad_right) {
            // Вращение вправо
            turret.manualRotateRaw(CALIBRATION_POWER);
            autoAimEnabled = false; // Отключаем auto-aim в режиме калибровки
            prevDpadRight = true;
            return; // Пропускаем остальную логику
        } else if (gamepad1 != null && prevDpadRight && !gamepad1.dpad_right) {
            // Dpad right отпущен - останавливаем и сбрасываем энкодер
            turret.manualRotateRaw(0.0);
            turret.resetEncoder();
            prevDpadRight = false;
            return;
        }

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