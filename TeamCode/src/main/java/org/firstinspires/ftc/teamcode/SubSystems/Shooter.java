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
    private Servo intakeStop;

    public enum HoodPosition {
        CLOSE(0.0),   // ≤30 cm - flat angle
        MIDDLE(0.7),  // ~70 cm - medium angle
        FAR(1.0);     // 150+ cm - high angle

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
    private static final double STOP_OPEN = 0.29;
    private static final double STOP_CLOSE = 0.0;
    private static final double INTAKE_STOP_ON = 0.9;   // Позиция во время стрельбы
    private static final double INTAKE_STOP_OFF = 1.0;  // Обычная позиция (не стреляем)
    private static final double SPIN_UP_TIME = 0.2;
    private static final double OPEN_STOP_TIME = 0.3;
    private static final double FEED_TIME = 2.5;

    public double kP = 0.007;
    public double kI = 0.0;
    public double kD = 0;
    public double kF = 0.00028;

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

    // Velocity deadzone - минимальное изменение расстояния для обновления velocity (см)
    private static final double VELOCITY_DEADZONE = 5.0;

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
    private boolean openStopExecuted = false;  // Флаг для OPEN_STOP state-entry
    private boolean feedExecuted = false;      // Флаг для FEED state-entry
    private boolean resetExecuted = false;     // Флаг для RESET state-entry
    private double lastHoodDistance = -1; // Последнее расстояние для hood (-1 = не инициализировано)
    private double lastVelocityDistance = -1; // Последнее расстояние для velocity (-1 = не инициализировано)

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        hood = hardwareMap.get(Servo.class, "shooterHood");
        shooterStop = hardwareMap.get(Servo.class, "shooterStop");
        intakeStop = hardwareMap.get(Servo.class, "intakeStop");

        // Настройка моторов для PID (такие же как в ShooterPIDTuner)
        // Motor1 в FORWARD - используется для чтения velocity в PID
        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        // Motor2 в REVERSE - для синхронного вращения
        shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        setHoodPosition(HoodPosition.CLOSE);

        // Обычные позиции когда не стреляем
        shooterStop.setPosition(STOP_CLOSE);
        intakeStop.setPosition(INTAKE_STOP_OFF);

        pidTimer.reset();
    }

    /**
     * Вычисляет динамическую позицию Hood на основе расстояния
     * Ступенчатая система углов:
     * ≤30 см → 0.0
     * ≤50 см → 0.5
     * ≤70 см → 0.7
     * ≤100 см → 0.9
     * ≤150+ см → 1.0
     */
    private double calculateHoodPosition(double distance) {
        if (distance <= 30.0) {
            return 0.0;
        } else if (distance <= 50.0) {
            return 0.5;
        } else if (distance <= 70.0) {
            return 0.7;
        } else if (distance <= 100.0) {
            return 0.9;
        } else {
            return 1.0; // 150+ cm
        }
    }

    /**
     * Вычисляет целевую velocity на основе расстояния до цели
     * Расстояние в см, возвращает velocity в ticks/sec
     * 0-30 см → 1300
     * 30-60 см → 1500
     * 60-150 см → 1800
     * 150+ см → 2000
     */
    private double calculateTargetVelocity(double distance) {
        if (distance <= 30.0) {
            return 1200.0;
        } else if (distance <= 60.0) {
            return 1500.0;
        } else if (distance <= 150.0) {
            return 1700.0;
        } else {
            return 2000.0;
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
     * Обновляет target velocity на основе расстояния до цели
     * Расстояние в см
     * Использует deadzone 5 см для предотвращения лишних изменений
     */
    public void updateVelocity(double distance) {
        if (distance > 0) {
            // Проверяем deadzone - обновляем только если изменение > 5 см
            if (lastVelocityDistance < 0 || Math.abs(distance - lastVelocityDistance) > VELOCITY_DEADZONE) {
                double newVelocity = calculateTargetVelocity(distance);

                // Обновляем velocity только если моторы уже крутятся или собираются крутиться
                // Не сбрасываем PID состояние если velocity не изменилась значительно
                if (Math.abs(newVelocity - targetVelocity) > 50.0) {
                    setTargetVelocity(newVelocity);
                }

                // Сохраняем последнее расстояние
                lastVelocityDistance = distance;
            }
        }
    }

    /**
     * Динамически обновляет Hood на основе расстояния от Vision
     * Вызывать в loop() для автоматической настройки
     */
    public void updateHoodDynamic(Vision vision) {
        double distance = 0;

        // Vision - если камера видит AprilTag
        if (vision != null && vision.hasTargetTag()) {
            distance = vision.getTargetDistance();
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
                // Запускаем моторы только если они не крутятся (для безопасности)
                if (targetVelocity == 0) {
                    on();
                }
                if (stateTimer.seconds() >= SPIN_UP_TIME) {
                    currentState = ShooterState.OPEN_STOP; //timer
                    stateTimer.reset();
                }
                break;

            case OPEN_STOP:
                // Открываем оба servo для стрельбы (ТОЛЬКО ОДИН РАЗ)
                if (!openStopExecuted) {
                    shooterStop.setPosition(STOP_OPEN);
                    intakeStop.setPosition(INTAKE_STOP_ON);
                    openStopExecuted = true;
                }
                if (stateTimer.seconds() >= OPEN_STOP_TIME) { //timer
                    currentState = ShooterState.FEED;
                    stateTimer.reset();
                    openStopExecuted = false; // Сброс для следующего раза
                }
                break;

            case FEED:
                // Включаем intake (ТОЛЬКО ОДИН РАЗ)
                if (!feedExecuted) {
                    intake.on();
                    feedExecuted = true;
                }
                if (stateTimer.seconds() >= FEED_TIME) { //timer
                    currentState = ShooterState.RESET;
                    stateTimer.reset();
                    feedExecuted = false; // Сброс для следующего раза
                }
                break;

            case RESET:
                // Возвращаем оба servo в обычные позиции (ТОЛЬКО ОДИН РАЗ)
                if (!resetExecuted) {
                    shooterStop.setPosition(STOP_CLOSE);
                    intakeStop.setPosition(INTAKE_STOP_OFF);
                    // НЕ выключаем shooter моторы - пусть крутятся постоянно в TeleOp
                    // off();
                    intake.off();
                    resetExecuted = true;
                }
                currentState = ShooterState.IDLE;
                resetExecuted = false; // Сброс для следующего раза
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
        targetVelocity = 2000; // ticks/sec - настроено через ShooterPIDTuner
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
        intakeStop.setPosition(INTAKE_STOP_OFF); // Обычная позиция
        setHoodPosition(HoodPosition.CLOSE);
        currentState = ShooterState.IDLE; // Сбрасываем FSM
        stateTimer.reset();
        lastHoodDistance = -1; // Сбрасываем deadzone tracking
        lastVelocityDistance = -1; // Сбрасываем velocity deadzone tracking
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

    public double getIntakeStopPosition() {
        return intakeStop.getPosition();
    }
}