package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem {

    private final DcMotorEx turretMotor;
    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
    private final Servo hoodServo;

    public static double TICKS_PER_DEGREE = 3.0;
    public static int TURRET_MIN_TICKS = (int)(-90 * TICKS_PER_DEGREE);
    public static int TURRET_MAX_TICKS = (int)(90 * TICKS_PER_DEGREE);

    public static double TURRET_P = 0.035;
    public static double TURRET_I = 0.0;
    public static double TURRET_D = 0.08;
    public static double TURRET_F = 0.015;

    public static double SHOOTER_P = 0.013;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 0.0048;

    public static double HOOD_DOWN = 0.0;
    public static double HOOD_MAX = 0.65;

    public static double DEFAULT_VELOCITY = 1150.0;
    public static double MIN_TARGETING_VELOCITY = 800.0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        hoodServo = hardwareMap.get(Servo.class, "hood");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setPositionPIDFCoefficients(TURRET_P);

        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(
                SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F
        );
        shooterMotor2.setVelocityPIDFCoefficients(
                shooterPIDF.p, shooterPIDF.i, shooterPIDF.d, shooterPIDF.f
        );

        setHoodPosition(HOOD_DOWN);
    }

    public void setTurretTargetTicks(int ticks) {
        int clampedTicks = Math.max(TURRET_MIN_TICKS, Math.min(TURRET_MAX_TICKS, ticks));
        turretMotor.setTargetPosition(clampedTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.8);
    }

    public int getTurretCurrentTicks() {
        return turretMotor.getCurrentPosition();
    }

    public void holdTurretPosition() {
        setTurretTargetTicks(getTurretCurrentTicks());
    }

    public void stopTurret() {
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setPower(0.0);
    }

    public void setTurretManualPower(double power) {
        if (turretMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        int currentTicks = getTurretCurrentTicks();

        if (currentTicks >= TURRET_MAX_TICKS && power > 0) {
            turretMotor.setPower(0);
        } else if (currentTicks <= TURRET_MIN_TICKS && power < 0) {
            turretMotor.setPower(0);
        } else {
            turretMotor.setPower(power);
        }
    }

    public void setHoodPosition(double position) {
        double clampedPosition = Math.max(HOOD_DOWN, Math.min(HOOD_MAX, position));
        hoodServo.setPosition(clampedPosition);
    }

    public void setShooterVelocity(double ticksPerSecond) {
        shooterMotor2.setVelocity(ticksPerSecond);
        shooterMotor1.setPower(ticksPerSecond / 2800.0);
    }

    public void setShooterDefaultVelocity() {
        setShooterVelocity(DEFAULT_VELOCITY);
    }

    public void stopShooter() {
        shooterMotor2.setVelocity(0.0);
        shooterMotor1.setPower(0.0);
    }
}