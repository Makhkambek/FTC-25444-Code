package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private DcMotor turretMotor;
    private Vision vision;
    private Localizer localizer;

    // PIDF коэффициенты (настроены для плавного tracking)
    public double kP = 0.035;   // Уменьшен для плавности
    public double kI = 0.0;
    public double kD = 0.008;   // Увеличен для демпфирования
    public double kF = 0.0015;  // Feedforward для преодоления трения

    private double integral = 0;
    private double lastError = 0;

    // Позиции турели в ГРАДУСАХ
    public static final double RED_TARGET = 45.0;    // Позиция для красной корзины
    public static final double BLUE_TARGET = -45.0;  // Позиция для синей корзины
    public static final double ZERO = 0.0;           // Центр

    // 270° диапазон: от -135° до +135°
    private static final double MAX_ANGLE = 135.0;   // Максимальный угол (градусы)
    private static final double MIN_ANGLE = -135.0;  // Минимальный угол (градусы)

    private static final double ANGLE_TOLERANCE = 2.0; // Допустимая ошибка в градусах

    // Сглаживание для плавного tracking
    private static final double SMOOTHING_FACTOR = 0.15; // 0.0-1.0, меньше = плавнее
    private double smoothedTargetAngle = 0.0;

    // Конвертация: тики ↔ градусы
    // ЭТУ КОНСТАНТУ настроишь экспериментально:
    // 1. Сбрось энкодер
    // 2. Поверни турель на 90°
    // 3. Посмотри тики (например, 900)
    // 4. TICKS_PER_DEGREE = 900 / 90 = 10.0
    public static double TICKS_PER_DEGREE  = 3.2; // Откалибровано: 108 ticks / 90° = 1.2

    // Manual control
    private double targetAngle = 0.0;
    private static final double MANUAL_STEP = 3.0; // Градусов за цикл при ручном управлении

    // Manual override (прямое управление без PID)
    private static final double OVERRIDE_POWER = 0.4; // Фиксированная мощность для аварийного режима

    // Координаты цели (корзины) для одометрии
    private Double goalX = null;
    private Double goalY = null;

    // Конструктор для TeleOp и Auto (с Vision и Localizer)
    public Turret(HardwareMap hardwareMap, Vision vision, Localizer localizer) {
        this.vision = vision;
        this.localizer = localizer;

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setDirection(DcMotor.Direction.REVERSE); // Инверт: вправо = +, влево = -
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetAngle = ZERO;
    }

    // Конструктор для Vision тестирования (только Vision, БЕЗ Localizer)
    public Turret(HardwareMap hardwareMap, Vision vision) {
        this.vision = vision;
        this.localizer = null; // Localizer не нужен для Vision tracking

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setDirection(DcMotor.Direction.REVERSE); // Инверт: вправо = +, влево = -
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetAngle = ZERO;
    }

    // Конструктор для базового тестирования (без Vision и Localizer)
    public Turret(HardwareMap hardwareMap) {
        this.vision = null;
        this.localizer = null;

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setDirection(DcMotor.Direction.REVERSE); // Инверт: вправо = +, влево = -
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetAngle = ZERO;
    }

    /**
     * Установить координаты цели (корзины) для автоприцеливания через одометрию
     */
    public void setGoalPosition(double x, double y) {
        this.goalX = x;
        this.goalY = y;
    }

    /**
     * Есть ли установленная цель?
     */
    public boolean hasGoal() {
        return goalX != null && goalY != null;
    }

    /**
     * Расстояние до цели (в см)
     */
    public double getDistanceToGoal() {
        if (goalX == null || goalY == null) {
            return 0.0;
        }

        double robotX = localizer.getX();
        double robotY = localizer.getY();

        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /**
     * Рассчитать угол турели для наведения на цель через одометрию
     *
     * Формула:
     * 1. targetDirection = atan2(yTarget - y, xTarget - x) — направление на цель в field coordinates
     * 2. turretAngle = targetDirection - robotHeading — угол турели относительно робота
     */
    private double calculateTargetAngle() {
        if (goalX == null || goalY == null || localizer == null) {
            return 0.0; // Нет цели или локалайзера - возвращаем центр
        }

        double robotX = localizer.getX();
        double robotY = localizer.getY();
        double robotHeadingRad = Math.toRadians(localizer.getHeading());

        // Направление на цель в field coordinates (радианы)
        double targetDirection = Math.atan2(goalY - robotY, goalX - robotX);

        // Угол турели = направление на цель - heading робота
        double turretAngleRad = normalizeRadians(targetDirection - robotHeadingRad);
        double turretAngle = Math.toDegrees(turretAngleRad);

        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, turretAngle));
    }

    /**
     * Нормализация угла в радианах в диапазон [-π, π]
     */
    private double normalizeRadians(double angleRad) {
        while (angleRad > Math.PI) angleRad -= 2 * Math.PI;
        while (angleRad < -Math.PI) angleRad += 2 * Math.PI;
        return angleRad;
    }

    /**
     * Нормализация угла в диапазон [-180, 180]
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Автоприцеливание с плавным сглаживанием:
     * 1. Если есть одометрия и установлена цель - используем calculateTargetAngle
     * 2. Если нет - используем Vision (yaw от AprilTag)
     *
     * Использует EMA (Exponential Moving Average) для плавного движения
     */
    public void autoAim() {
        double rawTarget = targetAngle; // Сохраняем текущий target как fallback

        // Приоритет 1: Odometry
        if (hasGoal() && localizer != null) {
            rawTarget = calculateTargetAngle();
        }
        // Приоритет 2: Vision
        else if (vision != null && vision.hasTargetTag()) {
            double yaw = vision.getTargetYaw();
            if (!Double.isNaN(yaw)) {
                rawTarget = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, yaw));
            }
        }

        // Плавное сглаживание через EMA (Exponential Moving Average)
        // smoothedTarget = smoothedTarget + factor * (rawTarget - smoothedTarget)
        smoothedTargetAngle += SMOOTHING_FACTOR * (rawTarget - smoothedTargetAngle);

        // Ограничиваем диапазон
        smoothedTargetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, smoothedTargetAngle));
        targetAngle = smoothedTargetAngle;

        // Применяем PIDF
        double currentAngle = getCurrentAngle();
        double power = calculatePIDF(targetAngle, currentAngle);
        turretMotor.setPower(power);
    }

    /**
     * Установить цель для автоприцеливания (RED_TARGET, BLUE_TARGET, или ZERO)
     */
    public void setAutoTarget(double angle) {
        targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
    }

    /**
     * Установить цель на основе альянса
     */
    public void setAutoTargetByAlliance(boolean isRedAlliance) {
        targetAngle = isRedAlliance ? RED_TARGET : BLUE_TARGET;
    }

    // Для сглаживания выходной мощности
    private static final double POWER_SMOOTHING = 0.3; // 0.0-1.0
    private double lastPower = 0.0;

    // Anti-windup лимит для интеграла
    private static final double INTEGRAL_LIMIT = 50.0;

    private double calculatePIDF(double target, double current) {
        double error = target - current;

        // Anti-windup: ограничиваем накопление интеграла
        integral += error;
        integral = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integral));

        // Сброс интеграла когда пересекаем ноль (помогает убрать overshoot)
        if (Math.signum(error) != Math.signum(lastError) && lastError != 0) {
            integral = 0;
        }

        double derivative = error - lastError;
        lastError = error;

        // PIDF = PID + Feedforward
        // kF помогает преодолеть статическое трение
        double feedforward = (Math.abs(error) > 1.0) ? kF * Math.signum(error) : 0;
        double rawPower = (kP * error) + (kI * integral) + (kD * derivative) + feedforward;

        // Сглаживание выходной мощности (убирает рывки)
        double smoothedPower = lastPower + POWER_SMOOTHING * (rawPower - lastPower);
        lastPower = smoothedPower;

        // Deadzone - если ошибка маленькая и мощность маленькая, не дёргаем мотор
        if (Math.abs(error) < 1.0 && Math.abs(smoothedPower) < 0.05) {
            return 0.0;
        }

        return Math.max(-0.8, Math.min(0.8, smoothedPower)); // Ограничиваем макс мощность
    }

    /**
     * Получить текущий угол турели в градусах
     */
    public double getCurrentAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    /**
     * Получить текущую позицию турели в тиках (для TurretTester)
     */
    public double getCurrentPosition() {
        return turretMotor.getCurrentPosition();
    }

    /**
     * Получить целевую позицию в тиках (для TurretTester)
     */
    public double getTargetPosition() {
        return angleToTicks(targetAngle);
    }

    /**
     * Конвертация: градусы → тики
     */
    private double angleToTicks(double angle) {
        return angle * TICKS_PER_DEGREE;
    }

    /**
     * Ручное управление турелью через джойстик (с PID)
     * Джойстик меняет целевой угол, PID держит турель
     */
    public void manualControl(double joystickInput) {
        if (Math.abs(joystickInput) > 0.1) { // Deadzone
            targetAngle += joystickInput * MANUAL_STEP;

            // Ограничиваем угол
            targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, targetAngle));
        }

        // PID держит турель в targetAngle
        double currentAngle = getCurrentAngle();
        double power = calculatePIDF(targetAngle, currentAngle);
        turretMotor.setPower(power);
    }

    /**
     * АВАРИЙНЫЙ режим: прямое управление мощностью БЕЗ PID
     * Используй если автоматика даст сбой
     *
     * @param direction: -1 = влево, 0 = стоп, +1 = вправо
     */
    public void manualOverride(double direction) {
        // Сбрасываем PID состояние чтобы не было "борьбы"
        integral = 0;
        lastError = 0;

        // Синхронизируем targetAngle с текущим углом
        // Чтобы когда вернемся к PID - турель не дернулась
        targetAngle = getCurrentAngle();

        // Прямое управление мощностью
        if (Math.abs(direction) > 0.1) {
            double power = Math.signum(direction) * OVERRIDE_POWER;

            // Защита от выхода за пределы
            double currentAngle = getCurrentAngle();
            if ((power > 0 && currentAngle >= MAX_ANGLE) ||
                (power < 0 && currentAngle <= MIN_ANGLE)) {
                turretMotor.setPower(0);
            } else {
                turretMotor.setPower(power);
            }
        } else {
            turretMotor.setPower(0);
        }
    }

    /**
     * Синхронизировать targetAngle с текущим углом
     * Вызывай перед переходом на manual control чтобы не было резкого движения
     */
    public void syncManualTarget() {
        double currentAngle = getCurrentAngle();
        targetAngle = currentAngle;
        smoothedTargetAngle = currentAngle;
        integral = 0;
        lastError = 0;
        lastPower = 0;
    }

    /**
     * Установить целевой угол напрямую (в градусах)
     */
    public void setTargetAngle(double angle) {
        targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
    }

    /**
     * Вернуть турель в центр (0)
     */
    public void returnToCenter() {
        setTargetAngle(ZERO);
    }

    /**
     * Турель в центральной позиции?
     */
    public boolean isCentered() {
        return Math.abs(getCurrentAngle()) < ANGLE_TOLERANCE;
    }

    /**
     * Получить текущий целевой угол
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Турель достигла целевого угла?
     */
    public boolean atTarget() {
        return Math.abs(targetAngle - getCurrentAngle()) < ANGLE_TOLERANCE;
    }

    /**
     * Получить текущую мощность мотора
     */
    public double getMotorPower() {
        return turretMotor.getPower();
    }

    /**
     * Турель отслеживает цель? (Vision или Odometry работают)
     */
    public boolean isTracking() {
        return (vision != null && vision.hasTargetTag()) || hasGoal();
    }

    public void setPIDF(double p, double i, double d, double f) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kF = f;
    }

    // Backward compatibility
    public void setPID(double p, double i, double d) {
        setPIDF(p, i, d, 0.0);
    }

    public void stop() {
        turretMotor.setPower(0);
        integral = 0;
        lastError = 0;
        lastPower = 0;
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle = ZERO;
        smoothedTargetAngle = ZERO;
        integral = 0;
        lastError = 0;
        lastPower = 0;
    }
}
