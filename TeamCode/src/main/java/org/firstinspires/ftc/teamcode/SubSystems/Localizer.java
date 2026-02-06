package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Localizer {
    private static Localizer instance = null;
    private GoBildaPinpointDriver pinpoint;

    private double x = 0;
    private double y = 0;
    private double heading = 0;
    private double rawHeading = 0;
    private double prevRawHeading = 0;

    public static Localizer getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new Localizer(hardwareMap);
        }
        return instance;
    }

    public static Localizer getInstance() {
        return instance;
    }

    private Localizer(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // NOTE: Do NOT call resetPosAndIMU() here!
        // Follower (Pedro Pathing) already initialized the Pinpoint hardware.
        // Calling reset here would override Follower's state and break setStartingPose()
        //
        // Configuration is NOT needed - Follower already configured Pinpoint via Constants.java
        // We only need to get the hardware reference to read heading
    }

    /**
     * ВЫЗЫВАТЬ ОДИН РАЗ В НАЧАЛЕ КАЖДОГО LOOP!
     */
    public void update() {
        pinpoint.update();

        x = pinpoint.getPosX(DistanceUnit.MM);
        y = pinpoint.getPosY(DistanceUnit.MM);

        double newRawHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        double deltaHeading = newRawHeading - prevRawHeading;
        prevRawHeading = newRawHeading;
        rawHeading = newRawHeading;

        if (Double.isFinite(deltaHeading)) {
            heading += deltaHeading;
        }

        // Нормализация
        while (heading > 180) heading -= 360;
        while (heading < -180) heading += 360;
    }

    /**
     * Обновить только heading (для DriveController)
     */
    public void updateHeadingOnly() {
        pinpoint.update();

        double newRawHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        double deltaHeading = newRawHeading - prevRawHeading;
        prevRawHeading = newRawHeading;
        rawHeading = newRawHeading;

        if (Double.isFinite(deltaHeading)) {
            heading += deltaHeading;
        }
    }

    public void reset() {
        // DO NOT reset Pinpoint hardware - Follower manages it
        // Only reset our internal tracking variables
        x = 0;
        y = 0;
        heading = 0;
        rawHeading = 0;
        prevRawHeading = 0;
    }

    /**
     * Установить позицию робота (для передачи из Auto в TeleOp)
     * x, y - в MM, heading - в градусах
     * Localizer сохраняет координаты через Singleton между Auto и TeleOp
     */
    public void setPosition(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.rawHeading = heading;
        this.prevRawHeading = heading;
    }

    // === GETTERS ===
    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
    public double getRawHeading() { return rawHeading; }

    public GoBildaPinpointDriver getPinpoint() { return pinpoint; }
}