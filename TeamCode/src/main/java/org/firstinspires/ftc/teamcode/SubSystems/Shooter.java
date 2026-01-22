package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    private DcMotor shooterMotor1, shooterMotor2;
    private Servo hood;
    private Servo shooterStop;

    public enum HoodPosition {
        CLOSE(0.0),
        MIDDLE(0.5),
        FAR(1.0);

        public final double position;

        HoodPosition(double position) {
            this.position = position;
        }
    }

    public enum ShooterState {
        IDLE,
        SPIN_UP,
        OPEN_STOP,
        FEED,
        RESET
    }

    private static final double SHOOTER_POWER = 1.0;
    private static final double STOP_OPEN = 1.0;
    private static final double STOP_CLOSE = 0.0;
    private static final double SPIN_UP_TIME = 0.2;
    private static final double OPEN_STOP_TIME = 0.3;
    private static final double FEED_TIME = 1.5;

    private HoodPosition currentHoodPosition = HoodPosition.MIDDLE;
    private ShooterState currentState = ShooterState.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotor.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");
        hood = hardwareMap.get(Servo.class, "shooterHood");
        shooterStop = hardwareMap.get(Servo.class, "shooterStop");

        // shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        setHoodPosition(HoodPosition.CLOSE);
        shooterStop.setPosition(STOP_CLOSE);
    }

    public void updateHood(double distance) {
        if (distance < 30) {
            setHoodPosition(HoodPosition.CLOSE);
        } else if (distance < 50) {
            setHoodPosition(HoodPosition.MIDDLE);
        } else if (distance < 100) {
            setHoodPosition(HoodPosition.FAR);
        }
    }

    public void startShoot() {
        if (currentState == ShooterState.IDLE) {
            currentState = ShooterState.SPIN_UP;
            stateTimer.reset();
        }
    }

    public void updateFSM(Intake intake) {
        switch (currentState) {
            case IDLE:
                break;

            case SPIN_UP:
                on();
                if (stateTimer.seconds() >= SPIN_UP_TIME) {
                    currentState = ShooterState.OPEN_STOP;
                    stateTimer.reset();
                }
                break;

            case OPEN_STOP:
                shooterStop.setPosition(STOP_OPEN);
                if (stateTimer.seconds() >= OPEN_STOP_TIME) {
                    currentState = ShooterState.FEED;
                    stateTimer.reset();
                }
                break;

            case FEED:
                intake.on();
                if (stateTimer.seconds() >= FEED_TIME) {
                    currentState = ShooterState.RESET;
                    stateTimer.reset();
                }
                break;

            case RESET:
                shooterStop.setPosition(STOP_CLOSE);
                off();
                intake.off();
                currentState = ShooterState.IDLE;
                break;
        }
    }

    public void on() {
        shooterMotor1.setPower(SHOOTER_POWER);
        shooterMotor2.setPower(SHOOTER_POWER);
    }

    public void off() {
        shooterMotor1.setPower(0.0);
        shooterMotor2.setPower(0.0);
    }

    public void setPower(double power) {
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }

    public void setHoodPosition(HoodPosition position) {
        if (position != null) {
            hood.setPosition(position.position);
            currentHoodPosition = position;
        }
    }

    public HoodPosition getCurrentHoodPosition() {
        return currentHoodPosition;
    }

    public ShooterState getCurrentState() {
        return currentState;
    }

    public boolean isRunning() {
        return shooterMotor1.getPower() > 0;
    }

    public boolean isShooting() {
        return currentState != ShooterState.IDLE;
    }

    public void reset() {
        // Полный сброс shooter в начальное состояние
        off(); // Выключаем моторы
        shooterStop.setPosition(STOP_CLOSE); // Закрываем stop
        setHoodPosition(HoodPosition.CLOSE); // Hood в начальную позицию
        currentState = ShooterState.IDLE; // Сбрасываем FSM
        stateTimer.reset();
    }

    // Testing methods
    public void setStopPosition(double position) {
        shooterStop.setPosition(position);
    }

    public void openStop() {
        shooterStop.setPosition(STOP_OPEN);
    }

    public void closeStop() {
        shooterStop.setPosition(STOP_CLOSE);
    }

    public double getStopPosition() {
        return shooterStop.getPosition();
    }
}