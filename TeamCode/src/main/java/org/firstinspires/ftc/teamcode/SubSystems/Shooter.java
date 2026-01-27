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
        MIDDLE(0.25),
        FAR(0.5);

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

    public double kP = 0.011;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kF = 0.00041;

    // Anti-windup limit для integral
    private static final double INTEGRAL_LIMIT = 100.0;

    // PID защита от спайков
    private static final double MIN_DELTA_TIME = 0.010; // минимум 10 мс между обновлениями
    private static final double MAX_DERIVATIVE = 500.0; // максимальное значение derivative

    // Защита от толчков
    private static final double OUTPUT_DEADBAND = 0.005; // минимальное изменение output для применения
    private static final double MAX_OUTPUT_CHANGE = 0.05; // максимальное изменение за один цикл (rate limiter)

    // Hood deadzone - минимальное изменение расстояния для обновления hood (см)
    private static final double HOOD_DEADZONE = 3.0;

    private double lastError = 0;
    private double integralSum = 0;
    private double targetVelocity = 0; // Целевая скорость в ticks/sec
    private ElapsedTime pidTimer = new ElapsedTime();

    // Сглаживание output для уменьшения дергания motor2
    private double smoothedOutput = 0;
    private static final double SMOOTHING_FACTOR = 0.9; // 0.0 = нет сглаживания, 1.0 = максимальное

    private HoodPosition currentHoodPosition = HoodPosition.MIDDLE;
    private ShooterState currentState = ShooterState.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();
    private double lastHoodDistance = -1; // Последнее расстояние для hood (-1 = не инициализировано)

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        hood = hardwareMap.get(Servo.class, "shooterHood");
        shooterStop = hardwareMap.get(Servo.class, "shooterStop");

        // Настройка моторов для PID
        // Motor1 БЕЗ REVERSE - для корректного чтения velocity
        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Motor2 в REVERSE направлении (для синхронного вращения)
        shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        setHoodPosition(HoodPosition.CLOSE);
        shooterStop.setPosition(STOP_CLOSE);

        pidTimer.reset();
    }

    /**
     * Вычисляет динамическую позицию Hood на основе расстояния
     * Расстояние в см, возвращает servo позицию (0.0 - 0.5)
     * 30 см → 0.0 (низкий угол)
     * 150 см → 0.4
     * 300 см → 0.5 (высокий угол)
     */
    private double calculateHoodPosition(double distance) {
        final double MIN_DISTANCE = 30.0;   // см
        final double MID_DISTANCE = 150.0;  // см
        final double MAX_DISTANCE = 300.0;  // см

        if (distance <= MIN_DISTANCE) {
            return 0.0; // Минимум
        } else if (distance <= MID_DISTANCE) {
            // 30-150: интерполяция 0.0 → 0.4
            double ratio = (distance - MIN_DISTANCE) / (MID_DISTANCE - MIN_DISTANCE);
            return ratio * 0.4;
        } else if (distance <= MAX_DISTANCE) {
            // 150-300: интерполяция 0.4 → 0.5
            double ratio = (distance - MID_DISTANCE) / (MAX_DISTANCE - MID_DISTANCE);
            return 0.4 + (ratio * 0.1);
        } else {
            return 0.5; // Максимум
        }
    }

    /**
     * Обновляет позицию Hood на основе расстояния до цели
     * Расстояние в см
     * Использует deadzone 3 см для предотвращения лишних движений
     */
    public void updateHood(double distance) {
        if (distance > 0) {
            // Проверяем deadzone - обновляем только если изменение > 3 см
            if (lastHoodDistance < 0 || Math.abs(distance - lastHoodDistance) > HOOD_DEADZONE) {
                double position = calculateHoodPosition(distance);
                hood.setPosition(position);

                // Обновляем currentHoodPosition для телеметрии
                if (position < 0.2) {
                    currentHoodPosition = HoodPosition.CLOSE;
                } else if (position < 0.45) {
                    currentHoodPosition = HoodPosition.MIDDLE;
                } else {
                    currentHoodPosition = HoodPosition.FAR;
                }

                // Сохраняем последнее расстояние
                lastHoodDistance = distance;
            }
        }
    }

    /**
     * Динамически обновляет Hood на основе расстояния
     * Приоритет: Vision (если видит AprilTag), затем Odometry
     * Вызывать в loop() для автоматической настройки
     */
    public void updateHoodDynamic(Turret turret, Vision vision) {
        double distance = 0;

        // Приоритет 1: Vision - если камера видит AprilTag
        if (vision != null && vision.hasTargetTag()) {
            distance = vision.getTargetDistance();
        }
        // Приоритет 2: Odometry - расстояние до goal из турели
        else if (turret != null && turret.hasGoal()) {
            distance = turret.getDistanceToGoal();
        }

        // Обновляем Hood если есть валидное расстояние
        if (distance > 0) {
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

        // Защита от слишком маленького deltaTime (предотвращает derivative spike)
        if (deltaTime < MIN_DELTA_TIME) {
            return; // Пропускаем этот цикл, слишком быстро
        }

        // Вычисляем derivative
        double derivative = (error - lastError) / deltaTime;

        // Ограничиваем derivative для предотвращения спайков
        derivative = Math.max(-MAX_DERIVATIVE, Math.min(MAX_DERIVATIVE, derivative));

        // Обновляем integral
        integralSum += error * deltaTime;

        // Anti-windup: ограничиваем integral для предотвращения overshoot
        integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));

        // PID calculation + Feedforward
        double output = (kP * error) + (kI * integralSum) + (kD * derivative) + (kF * targetVelocity);

        // Ограничение выхода
        output = Math.max(-1.0, Math.min(1.0, output));

        // Сглаживание output через EMA (Exponential Moving Average)
        double newSmoothedOutput = (SMOOTHING_FACTOR * smoothedOutput) + ((1.0 - SMOOTHING_FACTOR) * output);

        // Rate limiter - ограничиваем скорость изменения мощности
        double outputChange = newSmoothedOutput - smoothedOutput;
        if (Math.abs(outputChange) > MAX_OUTPUT_CHANGE) {
            outputChange = Math.signum(outputChange) * MAX_OUTPUT_CHANGE;
            newSmoothedOutput = smoothedOutput + outputChange;
        }

        // Deadband - применяем изменение только если оно достаточно большое
        if (Math.abs(outputChange) > OUTPUT_DEADBAND) {
            smoothedOutput = newSmoothedOutput;
        }

        // Устанавливаем мощность обоим моторам (используем сглаженный output)
        shooterMotor1.setPower(smoothedOutput);
        shooterMotor2.setPower(smoothedOutput);

        lastError = error;
    }

    public void on() {
        // Устанавливаем целевую скорость (например, максимальная)
        targetVelocity = 2200; // ticks/sec - настройте под ваши моторы
        lastError = 0;
        integralSum = 0;
        smoothedOutput = 0;
        pidTimer.reset();
    }

    public void off() {
        targetVelocity = 0;
        shooterMotor1.setPower(0.0);
        shooterMotor2.setPower(0.0);
        lastError = 0;
        integralSum = 0;
        smoothedOutput = 0;
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
        smoothedOutput = 0;
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

    public double getHoodServoPosition() {
        return hood.getPosition();
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
        setHoodPosition(HoodPosition.CLOSE);
        currentState = ShooterState.IDLE; // Сбрасываем FSM
        stateTimer.reset();
        lastHoodDistance = -1; // Сбрасываем deadzone tracking
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