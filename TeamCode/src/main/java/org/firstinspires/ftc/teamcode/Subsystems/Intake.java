package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    // Servos (names from your config)
    public final Servo intakeArmLeft;
    public final Servo intakeArmRight;
    public final Servo intakeRotate;
    public final Servo intakeTurn;
    public final Servo intakeGrab;

    private final DcMotor intakeMotor;

    private final Lift lift;
    private final Outtake outtake;

    public enum State {
        OPEN, OPENER, CLOSED, SPECIMEN, TRANSFERRING, INITIAL, IDLE, INTAKE_RUN, OUTTAKE_RUN
    }
    private State currentState = State.IDLE;
    private int subState = 0;
    private final ElapsedTime timer = new ElapsedTime();

    private static final double INTAKE_ARM_LEFT_OPEN = 0.221;
    private static final double INTAKE_ARM_RIGHT_OPEN = 0.219;
    private static final double INTAKE_ROTATE_OPEN = 0.24;
    private static final double INTAKE_GRAB_OPEN = 0.5;
    private static final double INTAKE_ARM_LEFT_DEFAULT = 0.471;
    private static final double INTAKE_ARM_RIGHT_DEFAULT = 0.469;
    private static final double INTAKE_ROTATE_CLOSED = 0.72;
    private static final double INTAKE_GRAB_CLOSED = 0.2;
    private static final double INTAKE_TURN_DEFAULT = 0.5;
    private static final double INTAKE_TURN_POS_2 = 0.8;
    private static final double INTAKE_TURN_POS_3 = 0.35;
    private static final double INTAKE_TURN_POS_4 = 0.68;

    public boolean isClosedComplete = false;
    public boolean isOpenComplete = false;
    public boolean isTransferComplete = false;
    public boolean isInitialPositionComplete = false;
    public boolean isSpecimenComplete = false;
    public boolean isOppenerComplete = false;

    public Intake(HardwareMap hw, Lift lift, Outtake outtake) {
        intakeArmLeft = hw.get(Servo.class, "intake_arm_left");
        intakeArmRight = hw.get(Servo.class, "intake_arm_right");
        intakeRotate = hw.get(Servo.class, "intake_rotate");
        intakeTurn = hw.get(Servo.class, "intake_turn");
        intakeGrab = hw.get(Servo.class, "intake_grab");
        intakeMotor = hw.get(DcMotor.class, "intake_motor");

        // reverse arms if needed (match repo)
        intakeArmLeft.setDirection(Servo.Direction.REVERSE);
        intakeArmRight.setDirection(Servo.Direction.REVERSE);
        intakeRotate.setDirection(Servo.Direction.REVERSE);

        this.lift = lift;
        this.outtake = outtake;

        setClosedPositions();
    }

    public void update() {
        switch (currentState) {
            case OPEN: executeOpen(); break;
            case OPENER: executeOppener(); break;
            case CLOSED: executeClosed(); break;
            case TRANSFERRING: executeTransfer(); break;
            case INITIAL: executeInitial(); break;
            case SPECIMEN: executeSpecimen(); break;
            case INTAKE_RUN:
                intakeMotor.setPower(1.0);
                break;
            case OUTTAKE_RUN:
                intakeMotor.setPower(-1.0);
                break;
            default:
                intakeMotor.setPower(0);
                break;
        }
    }

    private void executeOpen() {
        switch (subState) {
            case 0:
                intakeTurn.setPosition(INTAKE_TURN_DEFAULT);
                intakeArmLeft.setPosition(INTAKE_ARM_LEFT_DEFAULT);
                intakeArmRight.setPosition(INTAKE_ARM_RIGHT_DEFAULT);
                timer.reset();
                subState++;
                break;
            case 1:
                if (timer.seconds() > 0.2) {
                    intakeRotate.setPosition(INTAKE_ROTATE_OPEN);
                    intakeGrab.setPosition(INTAKE_GRAB_OPEN);
                    timer.reset();
                    subState++;
                }
                break;
            case 2:
                if (timer.seconds() > 0.15) {
                    currentState = State.IDLE;
                    isOpenComplete = true;
                    subState = 0;
                }
                break;
        }
    }

    private void executeOppener() {
        switch (subState) {
            case 0:
                intakeArmLeft.setPosition(INTAKE_ARM_LEFT_DEFAULT);
                intakeArmRight.setPosition(INTAKE_ARM_RIGHT_DEFAULT);
                timer.reset();
                subState++;
                break;
            case 1:
                if (timer.seconds() > 0.2) {
                    intakeRotate.setPosition(INTAKE_ROTATE_OPEN);
                    intakeGrab.setPosition(INTAKE_GRAB_OPEN);
                    timer.reset();
                    subState++;
                }
                break;
            case 2:
                if (timer.seconds() > 0.15) {
                    currentState = State.IDLE;
                    isOppenerComplete = true;
                    subState = 0;
                }
                break;
        }
    }

    private void executeClosed() {
        switch (subState) {
            case 0:
                intakeGrab.setPosition(INTAKE_GRAB_OPEN);
                intakeRotate.setPosition(INTAKE_ROTATE_OPEN);
                timer.reset();
                subState++;
                break;
            case 1:
                if (timer.seconds() > 0.15) {
                    intakeArmLeft.setPosition(INTAKE_ARM_LEFT_OPEN);
                    intakeArmRight.setPosition(INTAKE_ARM_RIGHT_OPEN);
                    timer.reset();
                    subState++;
                }
                break;
            case 2:
                if (timer.seconds() > 0.13) {
                    intakeGrab.setPosition(INTAKE_GRAB_CLOSED);
                    timer.reset();
                    subState++;
                }
                break;
            case 3:
                if (timer.seconds() > 0.2) {
                    intakeArmLeft.setPosition(INTAKE_ARM_LEFT_DEFAULT);
                    intakeArmRight.setPosition(INTAKE_ARM_RIGHT_DEFAULT);
                    timer.reset();
                    subState++;
                }
                break;
            case 4:
                if (timer.seconds() > 0.1) {
                    currentState = State.IDLE;
                    isClosedComplete = true;
                    subState = 0;
                }
                break;
        }
    }

    private void executeInitial() {
        switch (subState) {
            case 0:
                intakeArmLeft.setPosition(INTAKE_ARM_LEFT_DEFAULT);
                intakeArmRight.setPosition(INTAKE_ARM_RIGHT_DEFAULT);
                intakeGrab.setPosition(INTAKE_GRAB_CLOSED);
                timer.reset();
                subState++;
                break;
            case 1:
                if (timer.seconds() > 0.1) {
                    // retract intake motor a bit
                    intakeMotor.setPower(-0.2);
                    timer.reset();
                    subState++;
                }
                break;
            case 2:
                if (timer.seconds() > 0.1) {
                    intakeMotor.setPower(0);
                    currentState = State.IDLE;
                    isInitialPositionComplete = true;
                    subState = 0;
                }
                break;
        }
    }

    private void executeTransfer() {
        switch (subState) {
            case 0:
                intakeRotate.setPosition(INTAKE_ROTATE_CLOSED);
                intakeTurn.setPosition(INTAKE_TURN_DEFAULT);
                intakeMotor.setPower(-0.4);
                if (lift != null) lift.setState(Lift.State.GND);
                timer.reset();
                subState++;
                break;
            case 1:
                if (timer.seconds() > 0.05) {
                    intakeArmLeft.setPosition(INTAKE_ARM_LEFT_DEFAULT);
                    intakeArmRight.setPosition(INTAKE_ARM_RIGHT_DEFAULT);
                    if (outtake != null) {
                        outtake.outtakeLift.setPosition(0.12);
                        outtake.claw.setPosition(0.97);
                    }
                    timer.reset();
                    subState++;
                }
                break;
            case 2:
                if (timer.seconds() > 0.25) {
                    if (outtake != null) {
                        outtake.armLeft.setPosition(0.375);
                        outtake.armRight.setPosition(0.375);
                    }
                    timer.reset();
                    subState++;
                }
                break;
            case 3:
                if (timer.seconds() > 0.35) {
                    if (outtake != null) outtake.dropper.setPosition(Outtake.DROPPER_CLOSE);
                    timer.reset();
                    subState++;
                }
                break;
            case 4:
                if (timer.seconds() > 0.15) {
                    intakeGrab.setPosition(INTAKE_GRAB_OPEN);
                    timer.reset();
                    subState++;
                }
                break;
            case 5:
                if (timer.seconds() > 0.35) {
                    if (lift != null) lift.setState(Lift.State.HIGH);
                    intakeMotor.setPower(0);
                    intakeRotate.setPosition(INTAKE_ROTATE_OPEN);
                    intakeArmLeft.setPosition(INTAKE_ARM_LEFT_DEFAULT);
                    intakeArmRight.setPosition(INTAKE_ARM_RIGHT_DEFAULT);
                    if (outtake != null) outtake.setState(Outtake.State.SCORE);
                    timer.reset();
                    subState++;
                }
                break;
            case 6:
                if (timer.seconds() > 0.2) {
                    currentState = State.IDLE;
                    isTransferComplete = true;
                    subState = 0;
                }
                break;
        }
    }

    private void executeSpecimen() {
        switch (subState) {
            case 0:
                intakeMotor.setPower(-0.4);
                intakeTurn.setPosition(INTAKE_TURN_DEFAULT);
                intakeRotate.setPosition(INTAKE_ROTATE_CLOSED);
                intakeArmLeft.setPosition(INTAKE_ARM_LEFT_DEFAULT);
                intakeArmRight.setPosition(INTAKE_ARM_RIGHT_DEFAULT);
                if (outtake != null) {
                    outtake.outtakeLift.setPosition(0.12);
                    outtake.claw.setPosition(0.97);
                }
                timer.reset();
                subState++;
                break;
            case 1:
                if (timer.seconds() > 0.4) {
                    if (outtake != null) {
                        outtake.armLeft.setPosition(0.375);
                        outtake.armRight.setPosition(0.375);
                    }
                    timer.reset();
                    subState++;
                }
                break;
            case 2:
                if (timer.seconds() > 0.35) {
                    if (outtake != null) outtake.dropper.setPosition(Outtake.DROPPER_CLOSE);
                    timer.reset();
                    subState++;
                }
                break;
            case 3:
                if (timer.seconds() > 0.15) {
                    intakeGrab.setPosition(INTAKE_GRAB_OPEN);
                    timer.reset();
                    subState++;
                }
                break;
            case 4:
                if (timer.seconds() > 0.05) {
                    if (outtake != null) outtake.setState(Outtake.State.SCORE);
                    intakeRotate.setPosition(INTAKE_ROTATE_OPEN);
                    timer.reset();
                    subState++;
                }
                break;
            case 5:
                if (timer.seconds() > 0.15) {
                    currentState = State.IDLE;
                    isSpecimenComplete = true;
                    subState = 0;
                }
                break;
        }
    }

    public void setState(State s) {
        currentState = s;
        subState = 0;
        timer.reset();
        if (s == State.CLOSED) isClosedComplete = false;
        if (s == State.OPEN) isOpenComplete = false;
        if (s == State.TRANSFERRING) isTransferComplete = false;
        if (s == State.INITIAL) isInitialPositionComplete = false;
        if (s == State.SPECIMEN) isSpecimenComplete = false;
    }

    public State getState() { return currentState; }

    public void setClosedPositions() {
        intakeGrab.setPosition(INTAKE_GRAB_CLOSED);
        intakeArmLeft.setPosition(INTAKE_ARM_LEFT_DEFAULT);
        intakeArmRight.setPosition(INTAKE_ARM_RIGHT_DEFAULT);
        intakeRotate.setPosition(INTAKE_ROTATE_OPEN);
        intakeTurn.setPosition(INTAKE_TURN_DEFAULT);
        intakeMotor.setPower(0);
    }
}
