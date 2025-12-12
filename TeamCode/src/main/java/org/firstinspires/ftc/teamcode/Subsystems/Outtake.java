package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    public final Servo armLeft;
    public final Servo armRight;
    public final Servo claw;
    public final Servo outtakeLift;
    public final Servo hang1;
    public final Servo hang2;
    public final Servo dropper;

    public static final double ARM_LEFT_DEFAULT = 0.46;
    public static final double ARM_RIGHT_DEFAULT = 0.46;
    public static final double CLAW_GRAB = 0.9;
    public static final double CLAW_SCORE = 0.3;
    public static final double DROPPER_CLOSE = 0.25;
    public static final double DROPPER_OPEN = 0.55;
    public static final double OUTTAKE_LIFT_CLOSED = 0.0;
    public static final double OUTTAKE_LIFT_OPEN = 0.65;
    public static final double HANG_OPEN_1 = 0.4;
    public static final double HANG_OPEN_2 = 0.45;
    public static final double HANG_CLOSE_1 = 0.26;
    public static final double HANG_CLOSE_2 = 0.285;

    public enum State { IDLE, GRAB, DROP, SCORE, CLIPS_TAKE, CLIPS_PUT, PRE_LOAD, CLIPS_OPEN, HANG }

    private State currentState = State.IDLE;

    public Outtake(HardwareMap hw) {
        armLeft = hw.get(Servo.class, "arm_left");
        armRight = hw.get(Servo.class, "arm_right");
        claw = hw.get(Servo.class, "claw");
        outtakeLift = hw.get(Servo.class, "outtake_lift");
        hang1 = hw.get(Servo.class, "hang_1");
        hang2 = hw.get(Servo.class, "hang_2");
        dropper = hw.get(Servo.class, "dropper");

        armLeft.setDirection(Servo.Direction.REVERSE);
        armRight.setDirection(Servo.Direction.REVERSE);

        setPreloadPosition();
    }

    public void update() {
        switch (currentState) {
            case GRAB:
                claw.setPosition(CLAW_GRAB);
                outtakeLift.setPosition(OUTTAKE_LIFT_CLOSED);
                dropper.setPosition(DROPPER_OPEN);
                break;
            case DROP:
                dropper.setPosition(DROPPER_OPEN);
                armLeft.setPosition(ARM_LEFT_DEFAULT);
                armRight.setPosition(ARM_RIGHT_DEFAULT);
                break;
            case SCORE:
                armLeft.setPosition(ARM_LEFT_DEFAULT);
                armRight.setPosition(ARM_RIGHT_DEFAULT);
                claw.setPosition(CLAW_SCORE);
                outtakeLift.setPosition(OUTTAKE_LIFT_OPEN);
                dropper.setPosition(DROPPER_CLOSE);
                break;
            case HANG:
                armLeft.setPosition(ARM_LEFT_DEFAULT);
                armRight.setPosition(ARM_RIGHT_DEFAULT);
                claw.setPosition(CLAW_SCORE);
                hang1.setPosition(HANG_OPEN_1);
                hang2.setPosition(HANG_OPEN_2);
                break;
            default:
                break;
        }
    }

    public void setState(State s) {
        currentState = s;
    }

    public State getState() { return currentState; }

    public void setPreloadPosition() {
        armLeft.setPosition(ARM_LEFT_DEFAULT);
        armRight.setPosition(ARM_RIGHT_DEFAULT);
        claw.setPosition(CLAW_GRAB);
        dropper.setPosition(DROPPER_CLOSE);
        outtakeLift.setPosition(OUTTAKE_LIFT_CLOSED);
    }
}
