package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private DcMotor turretMotor;

    public double kP = 0.03;
    public double kI = 0.0;
    public double kD = 0.01;

    private double integral = 0;
    private double lastError = 0;

    // Позиции турели в тиках энкодера
    public static final double RED_TARGET = 100;    // Позиция для красной корзины
    public static final double BLUE_TARGET = -100;  // Позиция для синей корзины
    public static final double ZERO = 0;            // Центр

    private static final double MAX_POSITION = 200;  // Максимальная позиция (тики)
    private static final double MIN_POSITION = -200; // Минимальная позиция (тики)

    private static final double POSITION_TOLERANCE = 20.0; // Допустимая ошибка в тиках

    // Manual control
    private double targetPosition = 0.0;
    private static final double MANUAL_STEP = 15.0; // Тиков за цикл при ручном управлении

    // Manual override (прямое управление без PID)
    private static final double OVERRIDE_POWER = 0.4; // Фиксированная мощность для аварийного режима

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetPosition = ZERO;
    }


    /**
     * Автоприцеливание на заданную позицию (RED_TARGET или BLUE_TARGET)
     */
    public void autoAim() {
        double currentPosition = getCurrentPosition();
        double power = calculatePID(targetPosition, currentPosition);
        turretMotor.setPower(power);
    }

    /**
     * Установить цель для автоприцеливания (RED_TARGET, BLUE_TARGET, или ZERO)
     */
    public void setAutoTarget(double target) {
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, target));
    }

    /**
     * Установить цель на основе альянса
     */
    public void setAutoTargetByAlliance(boolean isRedAlliance) {
        targetPosition = isRedAlliance ? RED_TARGET : BLUE_TARGET;
    }

    private double calculatePID(double target, double current) {
        double error = target - current;

        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double power = (kP * error) + (kI * integral) + (kD * derivative);
        return Math.max(-1.0, Math.min(1.0, power));
    }

    public double getCurrentPosition() {
        return turretMotor.getCurrentPosition();
    }

    /**
     * Ручное управление турелью через джойстик (с PID)
     * Джойстик меняет целевую позицию, PID держит турель
     */
    public void manualControl(double joystickInput) {
        if (Math.abs(joystickInput) > 0.1) { // Deadzone
            targetPosition += joystickInput * MANUAL_STEP;

            // Ограничиваем позицию
            targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, targetPosition));
        }

        // PID держит турель в targetPosition
        double currentPosition = getCurrentPosition();
        double power = calculatePID(targetPosition, currentPosition);
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

        // Синхронизируем targetPosition с текущей позицией
        // Чтобы когда вернемся к PID - турель не дернулась
        targetPosition = getCurrentPosition();

        // Прямое управление мощностью
        if (Math.abs(direction) > 0.1) {
            double power = Math.signum(direction) * OVERRIDE_POWER;

            // Защита от выхода за пределы
            double currentPos = getCurrentPosition();
            if ((power > 0 && currentPos >= MAX_POSITION) ||
                (power < 0 && currentPos <= MIN_POSITION)) {
                turretMotor.setPower(0);
            } else {
                turretMotor.setPower(power);
            }
        } else {
            turretMotor.setPower(0);
        }
    }

    /**
     * Синхронизировать targetPosition с текущей позицией
     * Вызывай перед переходом на manual control чтобы не было резкого движения
     */
    public void syncManualTarget() {
        targetPosition = getCurrentPosition();
    }

    /**
     * Установить целевую позицию напрямую (в тиках)
     */
    public void setTargetPosition(double position) {
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, position));
    }

    /**
     * Вернуть турель в центр (0)
     */
    public void returnToCenter() {
        setTargetPosition(ZERO);
    }

    /**
     * Турель в центральной позиции?
     */
    public boolean isCentered() {
        return Math.abs(getCurrentPosition()) < POSITION_TOLERANCE;
    }

    /**
     * Получить текущую целевую позицию
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Турель достигла целевой позиции?
     */
    public boolean atTarget() {
        return Math.abs(targetPosition - getCurrentPosition()) < POSITION_TOLERANCE;
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
        targetPosition = ZERO;
    }
}