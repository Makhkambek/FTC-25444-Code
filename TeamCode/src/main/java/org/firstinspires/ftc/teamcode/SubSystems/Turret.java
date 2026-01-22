package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private DcMotor turretMotor;
    private Localizer localizer;
    private Vision vision;

    private double kP = 0.03;
    private double kI = 0.0;
    private double kD = 0.01;

    private double integral = 0;
    private double lastError = 0;

    private static final double MAX_ANGLE = 90;
    private static final double MIN_ANGLE = -90;

    // Энкодер тики на градус - КАЛИБРОВАТЬ!
    private static final double TICKS_PER_DEGREE = 10.0;

    // Координаты цели на поле - КАЛИБРОВАТЬ!
    private double targetX = 72.0;
    private double targetY = 144.0;

    private static final double ANGLE_TOLERANCE = 2.0;

    // Параметры сканирования
    private static final double SCAN_SPEED = 0.3;
    private boolean scanningRight = true;

    // Manual control
    private double manualTargetAngle = 0.0;
    private static final double MANUAL_ANGLE_STEP = 30.0; // градусов на единицу джойстика

    // Resetting state
    private boolean isResetting = false;

    public Turret(HardwareMap hardwareMap, Vision vision) {
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        localizer = Localizer.getInstance(hardwareMap);
        this.vision = vision;
        manualTargetAngle = 0.0;
        isResetting = false;
    }


    private double calculateTargetAngle() {
        double robotX = localizer.getX();
        double robotY = localizer.getY();
        double robotHeading = localizer.getHeading();

        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;

        double absoluteAngle = Math.toDegrees(Math.atan2(deltaX, deltaY));
        double turretAngle = normalizeAngle(absoluteAngle - robotHeading);

        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, turretAngle));
    }

    /**
     * Автоприцеливание с использованием Vision
     */
    public void autoAim() {
        // Если идет resetting - продолжаем возврат в центр
        if (isResetting) {
            continueResetting();
            return;
        }

        if (vision != null && vision.hasTargetTag()) {
            // Tag виден - отслеживаем его
            trackTarget();
        } else {
            // Tag не виден - сканируем
            scanForTarget();
        }
    }

    /**
     * Продолжает возврат в центр
     */
    private void continueResetting() {
        double currentAngle = getCurrentAngle();

        if (Math.abs(currentAngle) < ANGLE_TOLERANCE) {
            // Достигли центра - завершаем resetting
            isResetting = false;
            turretMotor.setPower(0);
            integral = 0;
            lastError = 0;
        } else {
            // Продолжаем движение к центру
            double power = calculatePID(0, currentAngle);
            turretMotor.setPower(power);
        }
    }

    /**
     * Отслеживание видимого AprilTag
     */
    private void trackTarget() {
        double yawError = vision.getTargetYaw();

        if (Double.isNaN(yawError)) {
            scanForTarget();
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

    /**
     * Сканирование в поисках цели
     */
    private void scanForTarget() {
        double currentAngle = getCurrentAngle();

        // Достигли правой границы - меняем направление
        if (currentAngle >= MAX_ANGLE - 5) {
            scanningRight = false;
        }
        // Достигли левой границы - меняем направление
        else if (currentAngle <= MIN_ANGLE + 5) {
            scanningRight = true;
        }

        // Двигаемся в текущем направлении
        double power = scanningRight ? SCAN_SPEED : -SCAN_SPEED;
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

    public void center() {
        double power = calculatePID(0, getCurrentAngle());
        turretMotor.setPower(power);
    }

    public void returnToCenter() {
        // Начинаем возврат на нулевую позицию
        isResetting = true;
        manualTargetAngle = 0.0;
        integral = 0;
        lastError = 0;
    }

    public boolean isCentered() {
        return Math.abs(getCurrentAngle()) < ANGLE_TOLERANCE;
    }

    public boolean isResettingToCenter() {
        return isResetting;
    }

    public void cancelReset() {
        isResetting = false;
    }

    public boolean isOnTarget() {
        double targetAngle = calculateTargetAngle();
        double currentAngle = getCurrentAngle();
        return Math.abs(targetAngle - currentAngle) < ANGLE_TOLERANCE;
    }

    public void setTargetPosition(double x, double y) {
        this.targetX = x;
        this.targetY = y;
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

    // Геттеры для телеметрии
    public double getRobotX() { return localizer.getX(); }
    public double getRobotY() { return localizer.getY(); }
    public double getRobotHeading() { return localizer.getHeading(); }

    public boolean isTracking() {
        return vision != null && vision.hasTargetTag();
    }

    public boolean isScanning() {
        return !isTracking();
    }
}