package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private DcMotor turretMotor;
    private Vision vision;
    private Localizer localizer;

    public double kP = 0.03;
    public double kI = 0.0;
    public double kD = 0.01;

    private double integral = 0;
    private double lastError = 0;

    // Позиции турели в ГРАДУСАХ
    public static final double RED_TARGET = 45.0;    // Позиция для красной корзины
    public static final double BLUE_TARGET = -45.0;  // Позиция для синей корзины
    public static final double ZERO = 0.0;           // Центр

    private static final double MAX_ANGLE = 90.0;    // Максимальный угол (градусы)
    private static final double MIN_ANGLE = -90.0;   // Минимальный угол (градусы)

    private static final double ANGLE_TOLERANCE = 5.0; // Допустимая ошибка в градусах

    // Конвертация: тики ↔ градусы
    // ЭТУ КОНСТАНТУ настроишь экспериментально:
    // 1. Сбрось энкодер
    // 2. Поверни турель на 90°
    // 3. Посмотри тики (например, 900)
    // 4. TICKS_PER_DEGREE = 900 / 90 = 10.0
    public static double TICKS_PER_DEGREE = 10.0; // Примерное значение, настрой при тесте

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
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetAngle = ZERO;
    }

    // Конструктор для TurretTester (без Vision и Localizer)
    public Turret(HardwareMap hardwareMap) {
        this.vision = null;
        this.localizer = null;

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
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
     */
    private double calculateTargetAngle() {
        if (goalX == null || goalY == null) {
            return 0.0; // Нет цели - возвращаем центр
        }

        double robotX = localizer.getX();
        double robotY = localizer.getY();
        double robotHeading = localizer.getHeading();

        // Разница координат
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

        // Угол на цель α = arctan2(ΔY, ΔX) в градусах
        double alpha = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Угол турели = Heading - α
        double turretAngle = normalizeAngle(robotHeading - alpha);

        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, turretAngle));
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
     * Автоприцеливание:
     * 1. Если Vision видит AprilTag - используем yaw
     * 2. Если нет - используем одометрию (calculateTargetAngle)
     */
    public void autoAim() {
        // Приоритет 1: Vision
        if (vision != null && vision.hasTargetTag()) {
            double yaw = vision.getTargetYaw();
            if (!Double.isNaN(yaw)) {
                targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, yaw));
            }
        }
        // Приоритет 2: Odometry
        else if (hasGoal()) {
            targetAngle = calculateTargetAngle();
        }

        // Применяем PID
        double currentAngle = getCurrentAngle();
        double power = calculatePID(targetAngle, currentAngle);
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

    private double calculatePID(double target, double current) {
        double error = target - current;

        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double power = (kP * error) + (kI * integral) + (kD * derivative);
        return Math.max(-1.0, Math.min(1.0, power));
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
        double power = calculatePID(targetAngle, currentAngle);
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
        targetAngle = getCurrentAngle();
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
     * Турель отслеживает цель? (Vision или Odometry работают)
     */
    public boolean isTracking() {
        return (vision != null && vision.hasTargetTag()) || hasGoal();
    }

    public void setPID(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    public void stop() {
        turretMotor.setPower(0);
        integral = 0;
        lastError = 0;
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle = ZERO;
    }
}
