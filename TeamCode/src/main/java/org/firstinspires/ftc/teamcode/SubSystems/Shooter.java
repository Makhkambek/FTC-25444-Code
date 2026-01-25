package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    private DcMotorEx shooterMotor1, shooterMotor2;
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

    // PID коэффициенты для shooter моторов
    public double kP = 0.005;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kF = 0.0;

    private double lastError = 0;
    private double integralSum = 0;
    private double targetVelocity = 0; // Целевая скорость в ticks/sec
    private ElapsedTime pidTimer = new ElapsedTime();

    private HoodPosition currentHoodPosition = HoodPosition.MIDDLE;
    private ShooterState currentState = ShooterState.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        hood = hardwareMap.get(Servo.class, "shooterHood");
        shooterStop = hardwareMap.get(Servo.class, "shooterStop");

        // Настройка моторов для PID
        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        setHoodPosition(HoodPosition.CLOSE);
        shooterStop.setPosition(STOP_CLOSE);

        pidTimer.reset();
    }

    /**
     * Обновляет позицию Hood на основе расстояния до цели
     * Расстояние в см
     */
    public void updateHood(double distance) {
        if (distance <= 0) {
            // Нет расстояния - используем позицию по умолчанию
            setHoodPosition(HoodPosition.MIDDLE);
        } else if (distance < 30) {
            setHoodPosition(HoodPosition.CLOSE);
        } else if (distance < 60) {
            setHoodPosition(HoodPosition.MIDDLE);
        } else {
            setHoodPosition(HoodPosition.FAR);
        }
    }

    /**
     * Динамически обновляет Hood на основе расстояния от турели
     * Вызывать в loop() для автоматической настройки
     */
    public void updateHoodDynamic(Turret turret) {
        if (turret != null && turret.hasGoal()) {
            double distance = turret.getDistanceToGoal();
            updateHood(distance);
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
                    currentState = ShooterState.OPEN_STOP; //timer
                    stateTimer.reset();
                }
                break;

            case OPEN_STOP:
                shooterStop.setPosition(STOP_OPEN);
                if (stateTimer.seconds() >= OPEN_STOP_TIME) { //timer
                    currentState = ShooterState.FEED;
                    stateTimer.reset();
                }
                break;

            case FEED:
                intake.on();
                if (stateTimer.seconds() >= FEED_TIME) { //timer
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

    /**
     * Обновляет PID для shooter моторов
     * Вызывать в каждом loop()
     */
    public void updatePID() {
        if (targetVelocity == 0) {
            shooterMotor1.setPower(0);
            shooterMotor2.setPower(0);
            return;
        }

        double currentVelocity = shooterMotor1.getVelocity();

        double error = targetVelocity - currentVelocity;

        double deltaTime = pidTimer.seconds();
        pidTimer.reset();

        if (deltaTime > 0) {
            double derivative = (error - lastError) / deltaTime;
            integralSum += error * deltaTime;

            double output = (kP * error) + (kI * integralSum) + (kD * derivative) + kF;

            // Ограничение выхода
            output = Math.max(-1.0, Math.min(1.0, output));

            // Устанавливаем мощность обоим моторам
            shooterMotor1.setPower(output);
            shooterMotor2.setPower(output); // Второй мотор просто повторяет первый

            lastError = error;
        }
    }

    public void on() {
        // Устанавливаем целевую скорость (например, максимальная)
        targetVelocity = 2200; // ticks/sec - настройте под ваши моторы
        lastError = 0;
        integralSum = 0;
        pidTimer.reset();
    }

    public void off() {
        targetVelocity = 0;
        shooterMotor1.setPower(0.0);
        shooterMotor2.setPower(0.0);
        lastError = 0;
        integralSum = 0;
    }

    public void setPower(double power) {
        // Прямое управление мощностью (без PID)
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
        lastError = 0;
        integralSum = 0;
        pidTimer.reset();
    }

    public double getCurrentVelocity() {
        return shooterMotor1.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
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