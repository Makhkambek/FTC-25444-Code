package org.firstinspires.ftc.teamcode.SubSystems;

import android.graphics.PointF;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    public static final PointF blueGoal = new PointF(12.0f, 138.0f);
    public static final PointF redGoal = new PointF(132.0f, 138.0f);

    private DcMotor turretMotor;
    private Localizer localizer;
    private Vision vision;

    public double kP = 0.03;
    public double kI = 0.0;
    public double kD = 0.01;

    private double integral = 0;
    private double lastError = 0;

    private static final double MAX_ANGLE = 90;
    private static final double MIN_ANGLE = -90;

    private static final double TICKS_PER_DEGREE = 10.0;

    private Double goalX = null;
    private Double goalY = null;

    private static final double ANGLE_TOLERANCE = 2.0;
    private static final double MOVEMENT_DEADZONE = 3.0; // градусов - минимальное изменение для реакции

    // Manual control
    private double manualTargetAngle = 0.0;
    private static final double MANUAL_ANGLE_STEP = 30.0; // градусов на единицу джойстика

    // Для отслеживания последнего целевого угла
    private double lastTargetAngle = 0.0;

    public Turret(HardwareMap hardwareMap, Vision vision) {
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        localizer = Localizer.getInstance(hardwareMap);
        this.vision = vision;
        manualTargetAngle = 0.0;
    }


    /**
     * Вычисляет угол турели для наведения на цель
     * Использует формулу: α = arctan2(Y_цели - Y_робота, X_цели - X_робота)
     * Angle_turret = Heading - α
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
     * Вычисляет расстояние до цели в см
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
     * Автоприцеливание с использованием Vision и одометров
     */
    public void autoAim() {
        if (vision != null && vision.hasTargetTag()) {
            // Tag виден - наводимся напрямую на тег через камеру
            trackTarget();
        } else if (goalX != null && goalY != null) {
            // Tag не виден, но есть координаты корзины - используем одометры
            trackWithOdometry();
        } else {
            // Нет цели - останавливаемся
            turretMotor.setPower(0);
        }
    }

    /**
     * Отслеживание цели с использованием одометров (когда камера не видит)
     */
    private void trackWithOdometry() {
        if (goalX == null || goalY == null) {
            turretMotor.setPower(0);
            return;
        }

        double targetAngle = calculateTargetAngle();

        // Deadzone: игнорируем маленькие изменения
        double angleDifference = Math.abs(targetAngle - lastTargetAngle);
        if (angleDifference < MOVEMENT_DEADZONE && lastTargetAngle != 0.0) {
            // Используем предыдущий целевой угол
            targetAngle = lastTargetAngle;
        } else {
            // Обновляем последний целевой угол
            lastTargetAngle = targetAngle;
        }

        double currentAngle = getCurrentAngle();
        double power = calculatePID(targetAngle, currentAngle);
        turretMotor.setPower(power);
    }

    /**
     * Отслеживание видимого AprilTag
     */
    private void trackTarget() {
        double yawError = vision.getTargetYaw();

        if (Double.isNaN(yawError)) {
            turretMotor.setPower(0);
            return;
        }

        // PID для отслеживания на основе yaw ошибки
        double currentAngle = getCurrentAngle();
        double targetAngle = currentAngle + yawError;

        // Ограничение углов
        targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, targetAngle));

        double power = calculatePID(targetAngle, currentAngle);
        turretMotor.setPower(power);
    }

    private double calculatePID(double target, double current) {
        double error = target - current;

        if (Math.abs(error) < ANGLE_TOLERANCE) {
            integral = 0;
            lastError = 0;
            return 0;
        }

        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double power = (kP * error) + (kI * integral) + (kD * derivative);
        return Math.max(-1.0, Math.min(1.0, power));
    }

    public double getCurrentAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public void manualControl(double joystickInput) {
        if (Math.abs(joystickInput) > 0) {
            manualTargetAngle += joystickInput * MANUAL_ANGLE_STEP * 0.02; // 0.02 для плавности

            // Ограничиваем целевой угол
            manualTargetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, manualTargetAngle));
        }

        // Используем PID для движения к целевому углу
        double currentAngle = getCurrentAngle();
        double power = calculatePID(manualTargetAngle, currentAngle);
        turretMotor.setPower(power);
    }

    public void syncManualTarget() {
        // Синхронизируем manual target с текущим углом
        manualTargetAngle = getCurrentAngle();
    }

    public void setTargetPosition(double position) {
        // Устанавливаем целевую позицию напрямую (для автономки)
        // position - это обороты/тики энкодера (например, 100 или -100)
        // PIDF мотора сам держит позицию
        manualTargetAngle = position;
    }

    public void returnToCenter() {
        // Устанавливаем целевую позицию на 0 (центр)
        setTargetPosition(0.0);
    }

    public boolean isCentered() {
        return Math.abs(getCurrentAngle()) < ANGLE_TOLERANCE;
    }

    /**
     * Установить координаты цели вручную
     */
    public void setGoalPosition(double x, double y) {
        this.goalX = x;
        this.goalY = y;
    }

    /**
     * Установить координаты цели на основе альянса
     */
    public void setGoalByAlliance(boolean isRedAlliance) {
        PointF goal = isRedAlliance ? redGoal : blueGoal;
        setGoalPosition(goal.x, goal.y);
    }

    public boolean hasGoal() {
        return goalX != null && goalY != null;
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
    }

    public boolean isTracking() {
        return vision != null && vision.hasTargetTag();
    }
}