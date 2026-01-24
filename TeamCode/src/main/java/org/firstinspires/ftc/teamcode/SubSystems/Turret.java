package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
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

    // Энкодер тики на градус - КАЛИБРОВАТЬ!
    private static final double TICKS_PER_DEGREE = 10.0;

    // Координаты цели на поле (сохраняются когда видим AprilTag)
    private Double goalX = null;
    private Double goalY = null;

    private static final double ANGLE_TOLERANCE = 2.0;

    // Manual control
    private double manualTargetAngle = 0.0;
    private static final double MANUAL_ANGLE_STEP = 30.0; // градусов на единицу джойстика

    public Turret(HardwareMap hardwareMap, Vision vision) {
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        localizer = Localizer.getInstance(hardwareMap);
        this.vision = vision;
        manualTargetAngle = 0.0;
    }


    private double calculateTargetAngle() {
        if (goalX == null || goalY == null) {
            return 0.0; // Нет цели - возвращаем центр
        }

        double robotX = localizer.getX();
        double robotY = localizer.getY();
        double robotHeading = localizer.getHeading();

        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

        double absoluteAngle = Math.toDegrees(Math.atan2(deltaX, deltaY));
        double turretAngle = normalizeAngle(absoluteAngle - robotHeading);

        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, turretAngle));
    }

    /**
     * Автоприцеливание с использованием Vision
     */
    public void autoAim() {
        if (vision != null && vision.hasTargetTag()) {
            // Tag виден - сохраняем координаты и отслеживаем через камеру
            saveGoalFromVision();
            trackTarget();
        } else if (goalX != null && goalY != null) {
            // Tag не виден, но есть сохраненная цель - используем одометры
            trackWithOdometry();
        } else {
            // Нет цели - останавливаемся
            turretMotor.setPower(0);
        }
    }

    /**
     * Сохраняет координаты цели на основе AprilTag и одометров
     */
    private void saveGoalFromVision() {
        if (vision == null || !vision.hasTargetTag()) {
            return;
        }

        // Получаем данные от Vision
        double distance = vision.getTargetDistance(); // в см
        double yaw = vision.getTargetYaw(); // угол от центра камеры

        if (distance <= 0 || Double.isNaN(yaw)) {
            return;
        }

        // Получаем текущую позицию робота
        double robotX = localizer.getX();
        double robotY = localizer.getY();
        double robotHeading = localizer.getHeading();
        double currentTurretAngle = getCurrentAngle();

        // Абсолютный угол к цели = heading робота + угол turret + yaw от камеры
        double absoluteAngleToGoal = robotHeading + currentTurretAngle + yaw;

        // Вычисляем координаты цели на поле
        goalX = robotX + distance * Math.sin(Math.toRadians(absoluteAngleToGoal));
        goalY = robotY + distance * Math.cos(Math.toRadians(absoluteAngleToGoal));
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
            // Обновляем целевой угол на основе джойстика
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

    public void setTargetAngle(double angle) {
        // Устанавливаем целевой угол (для автономки)
        manualTargetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
    }

    public void holdTargetAngle() {
        // Поддерживает установленный целевой угол
        double power = calculatePID(manualTargetAngle, getCurrentAngle());
        turretMotor.setPower(power);
    }

    public void center() {
        double power = calculatePID(0, getCurrentAngle());
        turretMotor.setPower(power);
    }

    public void returnToCenter() {
        // Устанавливаем целевой угол на 0 (центр)
        manualTargetAngle = 0.0;
        integral = 0;
        lastError = 0;
    }

    public boolean isCentered() {
        return Math.abs(getCurrentAngle()) < ANGLE_TOLERANCE;
    }

    public boolean isOnTarget() {
        if (!hasGoal()) {
            return false;
        }
        double targetAngle = calculateTargetAngle();
        double currentAngle = getCurrentAngle();
        return Math.abs(targetAngle - currentAngle) < ANGLE_TOLERANCE;
    }

    public void setGoalPosition(double x, double y) {
        this.goalX = x;
        this.goalY = y;
    }

    public void clearGoal() {
        this.goalX = null;
        this.goalY = null;
    }

    public boolean hasGoal() {
        return goalX != null && goalY != null;
    }

    public double getGoalX() {
        return goalX != null ? goalX : 0.0;
    }

    public double getGoalY() {
        return goalY != null ? goalY : 0.0;
    }

    public void setPID(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    public double getKP() {
        return kP;
    }

    public double getKI() {
        return kI;
    }

    public double getKD() {
        return kD;
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

    // Геттеры для телеметрии
    public double getRobotX() { return localizer.getX(); }
    public double getRobotY() { return localizer.getY(); }
    public double getRobotHeading() { return localizer.getHeading(); }

    public boolean isTracking() {
        return vision != null && vision.hasTargetTag();
    }
}