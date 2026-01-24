package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.SubSystems.Localizer;

@Config
public class HeadingController {
    private Localizer localizer;

    private double targetHeading = 0;
    private boolean isHeadingLocked = false;

    // PID коэффициенты (можно менять через Dashboard)
    public static double kP = 0.024;
    public static double kI = 0.0;
    public static double kD = 0.0003;
    public static double kF = 0.005;

    // PID переменные
    private double lastError = 0;
    private double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();

    public HeadingController(HardwareMap hardwareMap) {
        localizer = Localizer.getInstance(hardwareMap);
        timer.reset();
    }

    public void lockHeading() {
        if (!isHeadingLocked) {
            targetHeading = localizer.getHeading();
            isHeadingLocked = true;
            reset();
        }
    }

    public void unlockHeading() {
        isHeadingLocked = false;
    }

    public double calculateTurnPower() {
        if (!isHeadingLocked) {
            return 0;
        }

        double currentHeading = localizer.getHeading();
        double error = targetHeading - currentHeading;

        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();

        double output = (kP * error) + (kI * integralSum) + (kD * derivative) + (kF * error);

        lastError = error;
        timer.reset();

        return output;
    }

    public void reset() {
        targetHeading = 0;
        lastError = 0;
        integralSum = 0;
        timer.reset();
    }

    public boolean isHeadingLocked() {
        return isHeadingLocked;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public void debug(Telemetry telemetry) {
//        telemetry.addData("Target Heading", "%.2f°", targetHeading);
        telemetry.addData("Current Heading", "%.2f°", localizer.getHeading());
        telemetry.addData("Is Locked", isHeadingLocked);
//        telemetry.addData("Turn Power", "%.3f", calculateTurnPower());
    }
}