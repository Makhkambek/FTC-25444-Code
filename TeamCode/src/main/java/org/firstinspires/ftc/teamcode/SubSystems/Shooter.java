package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotor shooterMotor1, shooterMotor2;
    private Servo hood;

    public enum HoodPosition {
        CLOSE(0.0),   // Близкая цель - низкий угол
        MIDDLE(0.5),  // Средняя дистанция
        FAR(1.0);     // Далекая цель - высокий угол

        public final double position;
        HoodPosition(double position) {
            this.position = position;
        }
    }

    private static final double SHOOTER_POWER = 1.0;
    private HoodPosition currentHoodPosition = HoodPosition.MIDDLE;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotor.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");
        hood = hardwareMap.get(Servo.class, "shooterHood");

//        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        setHoodPosition(HoodPosition.CLOSE);
    }

    public void on() {
        shooterMotor1.setPower(SHOOTER_POWER);
        shooterMotor2.setPower(SHOOTER_POWER);
    }

    public void off() {
        shooterMotor1.setPower(0.0);
        shooterMotor2.setPower(0.0);
    }

    public void setPower(double power) {
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }

    public void setHoodPosition(HoodPosition position) {
        if (position != null) {
            hood.setPosition(position.position);
            currentHoodPosition = position;
        }
    }

    /**
     * Автоматически устанавливает Hood по данным от Vision
     */
    public void autoAdjustHood(Vision vision) {
        HoodPosition position = vision.getHoodPosition();
        if (position != null) {
            setHoodPosition(position);
        }
    }

    public HoodPosition getCurrentHoodPosition() {
        return currentHoodPosition;
    }

    public boolean isRunning() {
        return shooterMotor1.getPower() > 0;
    }
}