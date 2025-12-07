package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.arcrobotics.ftclib.command.SubsystemBase;

public class Lift extends SubsystemBase{
    private DcMotorEx firstLift;
    private DcMotorEx secondLift;
    private DcMotorEx thirdLift;

    public static final int GND = 0;
    public static final int LOW = 300;
    public static final int MID = 500;
    public static final int HIGH = 1000;

    private int ticks = GND;

    private double integralSum = 0;
    private double lastError = 0;

    private ElapsedTime timer = new ElapsedTime();

    public static double kP = 0.005;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public Lift(HardwareMap hardwareMap){
        firstLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        secondLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        thirdLift = hardwareMap.get(DcMotorEx.class, "middleLift");
        secondLift.setDirection(DcMotorEx.Direction.REVERSE);
        thirdLift.setDirection(DcMotorEx.Direction.REVERSE);
        firstLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        secondLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        thirdLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        resetEncoders();
    }

    private void resetEncoders(){
        firstLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        secondLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        thirdLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        firstLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        secondLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        thirdLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        ticks = 0;
    }

    public void setTicks(int newTicks){
        newTicks = Math.min(newTicks, HIGH);
        newTicks = Math.max(newTicks, GND);

        if (this.ticks != newTicks) {
            integralSum = 0;
            lastError = 0;
        }

        this.ticks = newTicks;
    }

    public int getCurrentPosition(){
        return firstLift.getCurrentPosition();
    }

    public void lift(double power){
        firstLift.setPower(power);
        secondLift.setPower(power);
        thirdLift.setPower(power);
    }

    public void fall(){
        firstLift.setPower(0);
        secondLift.setPower(0);
        thirdLift.setPower(0);
        resetEncoders();
    }

    public void update(){
        double position = firstLift.getCurrentPosition();
        double error = ticks - position;

        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();

        double output = (kP * error) + (kI * integralSum) + (kD * derivative) + kF;
        output = Math.max(-1, Math.min(output, 1));


        firstLift.setPower(output);
        secondLift.setPower(output);
        thirdLift.setPower(output);

        lastError = error;
        timer.reset();
    }

    public void periodic(){
        update();
    }

}
