package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem {

    private final Servo clutch_1;
    private final Servo clutch_2;

    // Adjust these values to match your claw positions
    private final double OPEN_POS  = 0.75;
    private final double CLOSE_POS = 0.25;

    public ClawSubsystem(HardwareMap hw) {
        clutch_1 = hw.get(Servo.class, "clutch_1");
        clutch_2 = hw.get(Servo.class, "clutch_2");

        // If your claws are mirrored, reverse one servo
        clutch_2.setDirection(Servo.Direction.REVERSE);
    }

    public void open() {
        clutch_1.setPosition(OPEN_POS);
        clutch_2.setPosition(OPEN_POS);
    }

    public void close() {
        clutch_1.setPosition(CLOSE_POS);
        clutch_2.setPosition(CLOSE_POS);
    }
}
