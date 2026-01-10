package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GateServoSubsystem {

    private final Servo gateServo;

    // Servo positions
    private static final double CLOSED_POSITION = 0.0;
    private static final double OPEN_POSITION = 0.9; // 90 degrees

    public GateServoSubsystem(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setPosition(CLOSED_POSITION);
    }

    public void open() {
        gateServo.setPosition(OPEN_POSITION);
    }

    public void close() {
        gateServo.setPosition(CLOSED_POSITION);
    }
}
