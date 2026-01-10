package org.firstinspires.ftc.teamcode.Controller;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Command.ShootCommand;

public class ShooterController {

    private final ShootCommand shootCommand;

    public ShooterController(ShootCommand shootCommand) {
        this.shootCommand = shootCommand;
    }

    public void update(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            shootCommand.start();
        }
        shootCommand.update();
    }
}
