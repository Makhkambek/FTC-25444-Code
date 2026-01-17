package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.SubSystems.Localizer;

@Config
public class HeadingController {
    private Localizer localizer;

    private double targetHeading = 0;
    private boolean isHeadingLocked = false;

    public static double kP = 0.024;
    public static double kI = 0.0;
    public static double kD = 0.0003;
    public static double kF = 0.005;    //should be good, but still check

    private final PIDController controller;

    public HeadingController(HardwareMap hardwareMap) {
        localizer = Localizer.getInstance(hardwareMap);
        controller = new PIDController(kP, kI, kD);
    }

    public void lockHeading() {
        if (!isHeadingLocked) {
            targetHeading = localizer.getHeading();
            isHeadingLocked = true;
            controller.reset();
        }
    }

    public void unlockHeading() {
        isHeadingLocked = false;
    }

    public double calculateTurnPower() {
        if (!isHeadingLocked) {
            return 0;
        }

        controller.setPID(kP, kI, kD);
        double currentHeading = localizer.getHeading();
        double error = currentHeading - targetHeading;

        double pidOutput = -controller.calculate(currentHeading, targetHeading);
        double feedforward = kF * error;

        return pidOutput + feedforward;
    }

    public void reset() {
        targetHeading = 0;
        controller.reset();
    }

    public boolean isHeadingLocked() {
        return isHeadingLocked;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public void debug(Telemetry telemetry) {
        telemetry.addData("Target Heading", "%.2f°", targetHeading);
        telemetry.addData("Current Heading", "%.2f°", localizer.getHeading());
        telemetry.addData("Is Locked", isHeadingLocked);
        telemetry.addData("Turn Power", "%.3f", calculateTurnPower());
    }
}