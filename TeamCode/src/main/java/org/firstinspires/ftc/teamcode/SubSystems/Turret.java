package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private DcMotor turretMotor;
    private Localizer localizer;

    // PID коэффициенты - КАЛИБРОВАТЬ!
    private double kP = 0.03;
    private double kI = 0.0;
    private double kD = 0.01;

    private double integral = 0;
    private double lastError = 0;

    // Лимиты турели
    private static final double MAX_ANGLE = 90;
    private static final double MIN_ANGLE = -90;

    // Энкодер тики на градус - КАЛИБРОВАТЬ!
    private static final double TICKS_PER_DEGREE = 10.0;

    // Координаты цели на поле - КАЛИБРОВАТЬ!
    private double targetX = 72.0;
    private double targetY = 144.0;

    // Deadzone
    private static final double ANGLE_TOLERANCE = 2.0;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Используем общий Localizer
        localizer = Localizer.getInstance(hardwareMap);
    }

    /**
     * Рассчитывает угол до цели
     */
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
     * Автоприцеливание
     */
    public void autoAim() {
        double targetAngle = calculateTargetAngle();
        double currentAngle = getCurrentAngle();
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

    public void manualControl(double power) {
        double currentAngle = getCurrentAngle();

        if ((currentAngle >= MAX_ANGLE && power > 0) ||
                (currentAngle <= MIN_ANGLE && power < 0)) {
            turretMotor.setPower(0);
            return;
        }

        turretMotor.setPower(power);
    }

    public void center() {
        double power = calculatePID(0, getCurrentAngle());
        turretMotor.setPower(power);
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
}