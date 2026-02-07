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

    // Vision correction weights for manual mode
    private static final double VISION_CORRECTION_WEIGHT_ACTIVE = 0.10;  // 10% когда джойстик активен
    private static final double VISION_CORRECTION_WEIGHT_IDLE = 0.25;    // 25% когда джойстик idle
    private static final double VISION_OFFSET_LIMIT = 5.0;  // Max градусов коррекции за loop
    private static final double VISION_SMOOTHING = 0.4;  // EMA сглаживание для vision offset (0.0-1.0)

    // Smoothed vision offset для уменьшения jittering
    private double smoothedVisionOffset = 0.0;

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

            // Получаем Vision correction если доступен
            double visionCorrection = 0.0;
            if (vision != null && vision.hasTargetTag()) {
                double rawOffset = vision.getTargetYaw();
                if (!Double.isNaN(rawOffset)) {
                    // Ограничиваем raw offset
                    double clampedOffset = Math.max(-VISION_OFFSET_LIMIT,
                                                    Math.min(VISION_OFFSET_LIMIT, rawOffset));

                    // EMA сглаживание для уменьшения jittering
                    smoothedVisionOffset += VISION_SMOOTHING * (clampedOffset - smoothedVisionOffset);

                    // Применяем weight
                    visionCorrection = smoothedVisionOffset * VISION_CORRECTION_WEIGHT_ACTIVE;
                }

                // Rumble feedback когда target visible
                gamepad.rumble(200);
            } else {
                // Vision lost - сбрасываем smoothed offset
                smoothedVisionOffset = 0.0;
            }

            // Применяем manual + vision correction
            turret.manualControlWithVisionCorrection(
                manualInput * MANUAL_SENSITIVITY,
                visionCorrection
            );

        } else {
            // Джойстик idle
            if (autoAimEnabled) {
                // Auto-aim mode
                turret.autoAim();
            } else {
                // Manual mode - maintain position с vision correction
                double visionCorrection = 0.0;
                if (vision != null && vision.hasTargetTag()) {
                    double rawOffset = vision.getTargetYaw();
                    if (!Double.isNaN(rawOffset)) {
                        // Ограничиваем raw offset
                        double clampedOffset = Math.max(-VISION_OFFSET_LIMIT,
                                                        Math.min(VISION_OFFSET_LIMIT, rawOffset));

                        // EMA сглаживание для уменьшения jittering
                        smoothedVisionOffset += VISION_SMOOTHING * (clampedOffset - smoothedVisionOffset);

                        // Применяем weight (более сильный для idle mode)
                        visionCorrection = smoothedVisionOffset * VISION_CORRECTION_WEIGHT_IDLE;
                    }
                } else {
                    // Vision lost - сбрасываем smoothed offset
                    smoothedVisionOffset = 0.0;
                }

                // Maintain с vision assist
                turret.manualControlWithVisionCorrection(0.0, visionCorrection);
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