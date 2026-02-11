package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
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
        FEED_SAMPLE,  // Encoder-based feeding of one sample
        PAUSE,        // Pause between samples
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

    // Time-based feeding parameters (FTC Dashboard tunable)
    // ВАЖНО: Порядок стрельбы = ближний → средний → дальний (intake крутится)
    public static double FEED_TIME_1 = 0.2;             // FIRST shot (closest ball to shooter, ~5cm)
    public static double FEED_TIME_2 = 0.3;             // SECOND shot (middle ball, ~10cm)
    public static double FEED_TIME_3 = 0.8;             // THIRD shot (farthest ball, ~15cm)
    public static double PAUSE_DURATION = 0.15;         // Seconds between shots (tunable)
    private static final int TARGET_SAMPLES = 3;        // Always shoot 3 samples

    public double kP = 0.007;
    public double kI = 0.0;
    public double kD = 0;
    public double kF = 0.00028;

    // Flywheel velocity formula coefficients (sigmoid)
    // V = L / (1 + e^(-(k * distance + b)))
    // Формула из Desmos - калибровка для робота (в дюймах)
    // ВАЖНО: Pedro Pathing использует дюймы (DistanceUnit.INCH)
    public static double VELOCITY_L = 2045.06422;      // Максимальная velocity (асимптота)
    public static double VELOCITY_K = 0.0195391;       // Скорость роста кривой
    public static double VELOCITY_B = -0.348709;       // Сдвиг (k*x0)

    // Velocity limits (clamp)
    private static final double MIN_VELOCITY = 0.0;        // Минимальная velocity
    private static final double MAX_VELOCITY = 2100.0;     // Увеличено для сигмоида
    public static double FLYWHEEL_OFFSET = 0.0;           // Offset для калибровки (tunable)

    // Hood angle formula coefficients (quadratic)
    // hoodAngle = A * distance² + B * distance + C
    // Формула из Desmos - калибровка для робота (в дюймах)
    // y = -0.0000985709x² + 0.0227599x - 0.584238
    public static double HOOD_COEFF_A = -0.0000985709;  // Коэффициент при x²
    public static double HOOD_COEFF_B = 0.0227599;      // Коэффициент при x
    public static double HOOD_COEFF_C = -0.584238;      // Свободный член

    // Hood angle limits
    private static final double MIN_HOOD_ANGLE = 0.0;   // Обновлено: min 0.0
    private static final double MAX_HOOD_ANGLE = 0.69;  // Обновлено: max 0.69 (velocity ~1800)
    public static double HOOD_OFFSET = 0.0;             // Offset для калибровки (tunable)

    // Anti-windup limit для integral
    private static final double INTEGRAL_LIMIT = 100.0;

    // PID защита от спайков
    private static final double MIN_DELTA_TIME = 0.010; // минимум 10 мс между обновлениями
    private static final double MAX_DERIVATIVE = 500.0; // максимальное значение derivative

    // Защита от толчков
    private static final double OUTPUT_DEADBAND = 0.005; // минимальное изменение output для применения
    private static final double MAX_OUTPUT_CHANGE = 0.05; // максимальное изменение за один цикл (rate limiter)

    // Hood deadzone - минимальное изменение расстояния для обновления hood (inches)
    // 10 дюймов = ~25 см - уменьшает jittering
    private static final double HOOD_DEADZONE = 10.0;

    // Velocity deadzone - минимальное изменение расстояния для обновления velocity (inches)
    // 5 дюймов = ~13 см
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
    private boolean manualStopOverride = false; // Ручное открытие shooterStop (приоритет над FSM)
    private double lastHoodDistance = -1; // Последнее расстояние для hood (-1 = не инициализировано)
    private double lastVelocityDistance = -1; // Последнее расстояние для velocity (-1 = не инициализировано)

    // Time-based feeding state tracking
    private int sampleCount = 0;               // Number of samples fed so far (0, 1, 2)
    private boolean feedSampleExecuted = false; // Flag for FEED_SAMPLE state-entry
    private boolean pauseExecuted = false;      // Flag for PAUSE state-entry
    private double[] feedTimes = new double[3]; // Array of feed times for each sample

    // Hood сглаживание для уменьшения jittering
    private static final double HOOD_SMOOTHING = 0.6;  // EMA factor для hood servo
    private double smoothedHoodPosition = 0.0;  // Текущая сглаженная позиция hood

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

        // Инициализируем smoothed hood position перед первым setHoodPosition
        smoothedHoodPosition = HoodPosition.CLOSE.position;
        setHoodPosition(HoodPosition.CLOSE);

        // Обычные позиции когда не стреляем
        shooterStop.setPosition(STOP_CLOSE);
        intakeStop.setPosition(INTAKE_STOP_OFF);

        // Инициализируем массив времен для time-based feeding
        updateFeedTimes();

        pidTimer.reset();
    }

    /**
     * Обновляет массив времен подачи из статических переменных
     * Вызывается при инициализации и перед каждым циклом стрельбы
     */
    private void updateFeedTimes() {
        feedTimes[0] = FEED_TIME_1;
        feedTimes[1] = FEED_TIME_2;
        feedTimes[2] = FEED_TIME_3;
    }

    /**
     * Helper: Clamp value between min and max
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Вычисляет target velocity на основе расстояния до цели
     * Использует сигмоидную функцию: V = L / (1 + e^(-(k*d + b)))
     * Формула из Desmos - калибровка для робота (в дюймах)
     *
     * @param distanceInches Расстояние до цели в дюймах (от Pedro Pathing odometry)
     * @return Target velocity в ticks/sec
     */
    private double calculateTargetVelocity(double distanceInches) {
        // Сигмоидная функция (логистическая регрессия)
        // V = L / (1 + e^(-(k * distance + b)))
        // distance уже в дюймах от Pedro Pathing
        double exponent = -(VELOCITY_K * distanceInches + VELOCITY_B);
        double velocity = VELOCITY_L / (1.0 + Math.exp(exponent));

        // Clamp к физическим лимитам + offset + velocity boost
        return clamp(velocity, MIN_VELOCITY, MAX_VELOCITY) + FLYWHEEL_OFFSET + 50.0;
    }

    /**
     * Вычисляет hood angle на основе расстояния до цели
     * Использует квадратичную формулу: angle = A*d² + B*d + C
     * Формула из Desmos - калибровка для робота (в дюймах)
     *
     * @param distanceInches Расстояние до цели в дюймах (от Pedro Pathing odometry)
     * @return Hood servo position (0.0 - 1.0)
     */
    private double calculateHoodAngle(double distanceInches) {
        // Квадратичная формула (парабола)
        // angle = A * distance² + B * distance + C
        // distance уже в дюймах от Pedro Pathing
        double angle = HOOD_COEFF_A * Math.pow(distanceInches, 2)
                     + HOOD_COEFF_B * distanceInches
                     + HOOD_COEFF_C;

        // Clamp к физическим лимитам servo + offset
        return clamp(angle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE) + HOOD_OFFSET;
    }

    /**
     * Обновляет позицию Hood на основе расстояния до цели
     * Расстояние в дюймах (от Pedro Pathing odometry)
     * Использует deadzone 10.0 units (~10 inches) для предотвращения лишних движений
     * Использует динамическую формулу вместо ступенчатой
     */
    public void updateHood(double distance) {
        if (distance > 0) {
            // Проверяем deadzone - обновляем только если изменение > 10 дюймов (~25 см)
            if (lastHoodDistance < 0 || Math.abs(distance - lastHoodDistance) > HOOD_DEADZONE) {
                // Используем квадратичную формулу из Desmos (парабола)
                double targetPosition = calculateHoodAngle(distance);

                // EMA сглаживание для уменьшения jittering
                if (lastHoodDistance < 0) {
                    // Первое обновление - устанавливаем напрямую
                    smoothedHoodPosition = targetPosition;
                } else {
                    // Применяем EMA фильтр
                    smoothedHoodPosition += HOOD_SMOOTHING * (targetPosition - smoothedHoodPosition);
                }

                hood.setPosition(smoothedHoodPosition);

                // Обновляем currentHoodPosition для телеметрии
                if (smoothedHoodPosition < 0.2) {
                    currentHoodPosition = HoodPosition.CLOSE;
                } else if (smoothedHoodPosition < 0.45) {
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

    public boolean isIdle() {
        return currentState == ShooterState.IDLE;
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
                    currentState = ShooterState.FEED_SAMPLE;
                    stateTimer.reset();
                    openStopExecuted = false; // Сброс для следующего раза

                    // Time-based feeding: reset sample counter and update feed times
                    sampleCount = 0;
                    updateFeedTimes(); // Обновляем времена на случай если изменили в FTC Dashboard
                }
                break;

            case FEED_SAMPLE:
                // Time-based feeding of one sample
                if (!feedSampleExecuted) {
                    // Turn on intake
                    intake.on();
                    feedSampleExecuted = true;
                }

                // Check timer - use time for CURRENT sample
                double currentFeedTime = feedTimes[sampleCount];

                if (stateTimer.seconds() >= currentFeedTime) {
                    // Sample fed successfully (by time)
                    intake.off();
                    sampleCount++;  // Increment counter (0→1, 1→2, 2→3)
                    feedSampleExecuted = false;

                    if (sampleCount < TARGET_SAMPLES) {
                        // More samples to feed (0→1→2) - go to pause
                        currentState = ShooterState.PAUSE;
                        stateTimer.reset();
                    } else {
                        // All 3 samples fed (sampleCount == 3) - go to reset
                        currentState = ShooterState.RESET;
                        stateTimer.reset();
                    }
                }
                break;

            case PAUSE:
                // Pause between samples (intake already off from FEED_SAMPLE)
                if (!pauseExecuted) {
                    pauseExecuted = true;
                }

                if (stateTimer.seconds() >= PAUSE_DURATION) {
                    // Pause complete - feed next sample
                    currentState = ShooterState.FEED_SAMPLE;
                    stateTimer.reset();
                    pauseExecuted = false;
                }
                break;

            case RESET:
                // Возвращаем оба servo в обычные позиции (ТОЛЬКО ОДИН РАЗ)
                if (!resetExecuted) {
                    // Закрываем shooterStop ТОЛЬКО если нет ручного override
                    if (!manualStopOverride) {
                        shooterStop.setPosition(STOP_CLOSE);
                    }
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
        // CRITICAL FIX: Only reset PID state if target velocity actually changed
        // Prevents PID reset spam when fallback sets same value every loop
        if (Math.abs(targetVelocity - velocity) > 1.0) {
            targetVelocity = velocity;
            lastError = 0;
            integralSum = 0;
            smoothedOutput = 0;
            pidTimer.reset();
            // CRITICAL FIX: Сбрасываем deadzone tracking чтобы следующий updateVelocity точно сработал
            // Без этого если fallback установит velocity=1000, а потом Vision увидит тег на близком расстоянии,
            // deadzone может блокировать обновление и velocity останется на 1000
            lastVelocityDistance = -1;
        } else {
            // Target unchanged - just update the value without resetting PID
            targetVelocity = velocity;
        }
    }

    public double getCurrentVelocity() {
        return shooterMotor1.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setHoodPosition(HoodPosition position) {
        if (position != null) {
            // Применяем сглаживание и для manual override
            smoothedHoodPosition += HOOD_SMOOTHING * (position.position - smoothedHoodPosition);
            hood.setPosition(smoothedHoodPosition);
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

    public int getSampleCount() {
        return sampleCount;
    }

    public void reset() {
        // Полный сброс shooter в начальное состояние
        off(); // Выключаем моторы
        manualStopOverride = false; // Сбрасываем manual override
        shooterStop.setPosition(STOP_CLOSE); // Закрываем stop
        intakeStop.setPosition(INTAKE_STOP_OFF); // Обычная позиция
        setHoodPosition(HoodPosition.CLOSE);
        currentState = ShooterState.IDLE; // Сбрасываем FSM
        stateTimer.reset();
        lastHoodDistance = -1; // Сбрасываем deadzone tracking
        lastVelocityDistance = -1; // Сбрасываем velocity deadzone tracking
    }

    /**
     * Ручное управление shooterStop (приоритет над FSM)
     * Когда активен - shooterStop остается открытым, FSM не может закрыть
     *
     * @param enabled true - держать открытым, false - FSM работает как обычно
     */
    public void setManualStopOverride(boolean enabled) {
        manualStopOverride = enabled;

        if (manualStopOverride) {
            // Немедленно открываем shooterStop
            shooterStop.setPosition(STOP_OPEN);
        }
        // Если disabled - FSM закроет shooterStop когда нужно (в RESET state)
    }

    /**
     * Принудительно закрывает shooterStop (для dpad_down)
     */
    public void forceCloseStop() {
        shooterStop.setPosition(STOP_CLOSE);
    }

    /**
     * Устанавливает позицию hood напрямую (для fallback когда нет Vision tag)
     * @param position позиция servo (0.0-1.0)
     */
    public void setHoodPosition(double position) {
        hood.setPosition(clamp(position, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE) + HOOD_OFFSET);
        // CRITICAL FIX: Сбрасываем deadzone tracking чтобы следующий updateHood точно сработал
        // Без этого если fallback установит hood=0, а потом Vision увидит тег на близком расстоянии,
        // deadzone может блокировать обновление и hood останется на 0
        lastHoodDistance = -1;
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