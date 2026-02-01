package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class Turret {
    private DcMotor turretMotor;
    private Vision vision;
    private Localizer localizer;
    private Follower follower; // Pedro Pathing Follower (альтернатива Localizer)

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

    // Координаты цели (корзины) для одометрии - Legacy (deprecated)
    private Double goalX = null;
    private Double goalY = null;

    // Goal Pose - современный способ задания цели через Pedro Pathing Pose
    private Pose goalPose = null;

    // Debug переменные для телеметрии
    public double debugRobotX = 0;
    public double debugRobotY = 0;
    public double debugTargetX = 0;
    public double debugTargetY = 0;
    public double debugDeltaX = 0;
    public double debugDeltaY = 0;
    public double debugTargetDirectionDeg = 0;
    public double debugRobotHeadingDeg = 0;
    public double debugCalculatedAngleDeg = 0;

    // Конструктор для TeleOp и Auto (с Vision и Localizer)
    public Turret(HardwareMap hardwareMap, Vision vision, Localizer localizer) {
        this.vision = vision;
        this.localizer = localizer;
        this.follower = null;

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setDirection(DcMotor.Direction.REVERSE); // Инверт: вправо = +, влево = -
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetAngle = ZERO;
    }

    // Конструктор для TeleOp с Pedro Pathing Follower (правильная одометрия!)
    public Turret(HardwareMap hardwareMap, Vision vision, Follower follower) {
        this.vision = vision;
        this.follower = follower;
        this.localizer = null; // Используем follower вместо localizer

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
        this.follower = null;

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
        this.follower = null;

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setDirection(DcMotor.Direction.REVERSE); // Инверт: вправо = +, влево = -
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetAngle = ZERO;
    }

    /**
     * Установить координаты цели (корзины) для автоприцеливания через одометрию
     * Legacy метод - используй setGoalPose() для Pedro Pathing
     */
    public void setGoalPosition(double x, double y) {
        this.goalX = x;
        this.goalY = y;
    }

    /**
     * Установить Goal Pose для турели (современный способ через Pedro Pathing)
     */
    public void setGoalPose(Pose goal) {
        this.goalPose = goal;
        // Синхронизируем со старыми переменными для совместимости
        if (goal != null) {
            this.goalX = goal.getX();
            this.goalY = goal.getY();
        }
    }

    /**
     * Есть ли установленная цель?
     */
    public boolean hasGoal() {
        return goalPose != null || (goalX != null && goalY != null);
    }

    /**
     * Расстояние до цели (в см)
     * Использует Pose объекты для вычислений
     */
    public double getDistanceToGoal() {
        if (goalPose == null && (goalX == null || goalY == null)) {
            return 0.0;
        }

        Pose currentPose;

        // Получаем текущую позицию робота
        if (follower != null) {
            currentPose = follower.getPose(); // Используем существующий Pose!
        } else if (localizer != null) {
            currentPose = new Pose(localizer.getX(), localizer.getY(), 0);
        } else {
            return 0.0; // Нет локализации
        }

        // Используем goalPose если доступен
        double targetX, targetY;
        if (goalPose != null) {
            targetX = goalPose.getX();
            targetY = goalPose.getY();
        } else {
            targetX = goalX;
            targetY = goalY;
        }

        double deltaX = targetX - currentPose.getX();
        double deltaY = targetY - currentPose.getY();

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /**
     * Рассчитать угол турели для наведения на цель через одометрию
     *
     * PEDRO PATHING COORDINATE SYSTEM:
     * - X = Forward/Backward (vertical axis on visualizer)
     * - Y = Left/Right (horizontal axis on visualizer)
     * - Source of truth: follower.getPose()
     *
     * MATHEMATICAL STEPS:
     * 1. Calculate field angle: worldAngle = atan2(deltaY, deltaX)
     * 2. Convert to robot-relative: relativeAngle = worldAngle - robotHeading
     * 3. Normalize to [-π, π] using atan2 trick
     * 4. Convert to degrees
     * 5. Clamp to physical limits
     *
     * IMPORTANT: This runs continuously every loop, even when joysticks are idle
     */
    private double calculateTargetAngle() {
        // Check if we have a goal
        if (goalPose == null && (goalX == null || goalY == null)) {
            return 0.0; // No target - return center
        }

        // STEP 0: Get current pose from follower (source of truth)
        Pose currentPose;
        if (follower != null) {
            currentPose = follower.getPose(); // Fresh coordinates from Pinpoint every loop!
        } else if (localizer != null) {
            // Fallback to localizer
            double heading = Math.toRadians(localizer.getHeading());
            currentPose = new Pose(localizer.getX(), localizer.getY(), heading);
        } else {
            return 0.0; // No localization available
        }

        // Get target coordinates
        double targetX, targetY;
        if (goalPose != null) {
            targetX = goalPose.getX();
            targetY = goalPose.getY();
        } else {
            targetX = goalX;
            targetY = goalY;
        }

        // STEP 1: Calculate field angle (world angle) to target
        // Pedro Pathing: X=forward/backward, Y=left/right
        double deltaX = currentPose.getX() - targetX; //currentPose.getX() - targetX reversed
        double deltaY = targetY - currentPose.getY(); //targetY - currentPose.getY();
        double worldAngle = Math.atan2(deltaY, deltaX);

        // STEP 2: Convert to robot-relative angle
        double robotHeading = currentPose.getHeading();
        double relativeAngle = worldAngle - robotHeading;

        // STEP 3: Normalize to [-π, π] using atan2 trick (shortest path)
        relativeAngle = normalizeAngle(relativeAngle);

        // STEP 4: Convert to degrees and invert sign
        double turretAngleDeg = -Math.toDegrees(relativeAngle);

        // STEP 5: Add offset - when moving forward (X increases), turret angle should increase
        turretAngleDeg = -turretAngleDeg + 90.0;

        // DEBUG: Save values for telemetry
        debugRobotX = currentPose.getX();
        debugRobotY = currentPose.getY();
        debugTargetX = targetX;
        debugTargetY = targetY;
        debugDeltaX = deltaX;
        debugDeltaY = deltaY;

        // Convert angles to [0°, 360°] for display convenience
        debugTargetDirectionDeg = Math.toDegrees(worldAngle);
        while (debugTargetDirectionDeg < 0) debugTargetDirectionDeg += 360;
        while (debugTargetDirectionDeg >= 360) debugTargetDirectionDeg -= 360;

        debugRobotHeadingDeg = Math.toDegrees(robotHeading);
        while (debugRobotHeadingDeg < 0) debugRobotHeadingDeg += 360;
        while (debugRobotHeadingDeg >= 360) debugRobotHeadingDeg -= 360;

        debugCalculatedAngleDeg = turretAngleDeg; // This stays in [-135°, 135°]

        // STEP 5: Clamp to physical turret limits (±135°)
        turretAngleDeg = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, turretAngleDeg));

        return turretAngleDeg;
    }

    /**
     * Normalize angle to [-π, π] using atan2 trick
     * This ensures the shortest path and prevents "snapping" at 0°/360° boundary
     */
    private double normalizeAngle(double angleRad) {
        return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
    }

    /**
     * Автоприцеливание:
     * 1. Если есть одометрия и установлена цель - используем calculateTargetAngle (БЕЗ сглаживания!)
     * 2. Если нет - используем Vision (yaw от AprilTag) с EMA сглаживанием
     *
     * ВАЖНО: Odometry tracking БЕЗ сглаживания для мгновенной реакции на движение робота!
     */
    public void autoAim() {
        boolean usingOdometry = false;

        // Приоритет 1: Odometry (follower ИЛИ localizer)
        if (hasGoal() && (follower != null || localizer != null)) {
            // Odometry - прямое значение БЕЗ сглаживания!
            targetAngle = calculateTargetAngle();
            smoothedTargetAngle = targetAngle; // Синхронизируем smoothed с raw
            usingOdometry = true;
        }
        // Приоритет 2: Vision (с EMA сглаживанием для стабильности)
        else if (vision != null && vision.hasTargetTag()) {
            double yaw = vision.getTargetYaw();
            if (!Double.isNaN(yaw)) {
                double rawTarget = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, yaw));

                // Плавное сглаживание ТОЛЬКО для Vision (AprilTag может прыгать)
                smoothedTargetAngle += SMOOTHING_FACTOR * (rawTarget - smoothedTargetAngle);
                smoothedTargetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, smoothedTargetAngle));
                targetAngle = smoothedTargetAngle;
            }
        }

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
