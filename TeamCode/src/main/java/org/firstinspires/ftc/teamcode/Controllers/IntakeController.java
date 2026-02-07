package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeController {
    public Gamepad gamepad;
    private Intake intake;

    public IntakeController(Gamepad gamepad, Intake intake) {
        this.gamepad = gamepad;
        this.intake = intake;
    }


    public void update() {
        if (gamepad == null) return;

        // Увеличенный deadzone (0.5) для предотвращения ложных срабатываний от дрейфа триггера
        if (gamepad.right_trigger > 0.5) {
            intake.on();
        } else if (gamepad.left_trigger > 0.5) {
            intake.reverse();
        } else {
            intake.off();
        }
    }
}