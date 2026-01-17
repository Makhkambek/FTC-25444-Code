package org.firstinspires.ftc.teamcode.SubSystems;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Localizer {
    private static Localizer instance = null;
    private GoBildaPinpointDriver pinpoint;

    // Позиция робота
    private double x = 0;
    private double y = 0;
    private double heading = 0;
    private double rawHeading = 0;
    private double prevRawHeading = 0;

    // Singleton - только один экземпляр
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
        pinpoint.setOffsets(-107.95, -63.5); // Твои оффсеты
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD); //change it
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
    }

    /**
     * ВЫЗЫВАТЬ ОДИН РАЗ В НАЧАЛЕ КАЖДОГО LOOP!
     */
    public void update() {
        pinpoint.update();

        x = pinpoint.getPosX();
        y = pinpoint.getPosY();

        double newRawHeading = Math.toDegrees(pinpoint.getHeading());
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
        pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);

        double newRawHeading = Math.toDegrees(pinpoint.getHeading());
        double deltaHeading = newRawHeading - prevRawHeading;
        prevRawHeading = newRawHeading;
        rawHeading = newRawHeading;

        if (Double.isFinite(deltaHeading)) {
            heading += deltaHeading;
        }
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
        x = 0;
        y = 0;
        heading = 0;
        rawHeading = 0;
        prevRawHeading = 0;
    }

    // === GETTERS ===
    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
    public double getRawHeading() { return rawHeading; }

    public GoBildaPinpointDriver getPinpoint() { return pinpoint; }
}