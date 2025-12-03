package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem {

    private final Servo push_1;
    private final Servo push_2;

    // Full range (adjust after testing)
    private final double UP_POS   = 1.0;
    private final double DOWN_POS = 0.0;

    public ArmSubsystem(HardwareMap hw) {
        push_1 = hw.get(Servo.class, "push_1");
        push_2 = hw.get(Servo.class, "push_2");

        // If the servos move opposite directions, reverse one
        push_2.setDirection(Servo.Direction.REVERSE);
    }

    public void up() {
        push_1.setPosition(UP_POS);
        push_2.setPosition(UP_POS);
    }

    public void down() {
        push_1.setPosition(DOWN_POS);
        push_2.setPosition(DOWN_POS);
    }
}
