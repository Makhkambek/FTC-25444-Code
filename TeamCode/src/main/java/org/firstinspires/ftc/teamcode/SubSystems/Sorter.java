package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class Sorter {
    private Servo servo0, servo1, servo2;
    private ColorSensor colorSensor0, colorSensor1, colorSensor2;

    public enum ShootSequence {
        PPG, // Purple → Purple → Green
        PGP, // Purple → Green → Purple
        GPP  // Green → Purple → Purple
    }

    private enum BallColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    private static final double SERVO_RETRACT = 0.0;
    private static final double SERVO_PUSH = 1.0;

    private static final int COLOR_THRESHOLD = 100;

    // Текущая конфигурация барабана
    private BallColor[] drumConfiguration = new BallColor[3];

    public Sorter(HardwareMap hardwareMap) {
        servo0 = hardwareMap.get(Servo.class, "sorterServo0");
        servo1 = hardwareMap.get(Servo.class, "sorterServo1");
        servo2 = hardwareMap.get(Servo.class, "sorterServo2");

        colorSensor0 = hardwareMap.get(ColorSensor.class, "colorSensor0");
        colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "colorSensor2");

        retractAll();
    }


    public void scanBalls() {
        drumConfiguration[0] = detectColor(colorSensor0);
        drumConfiguration[1] = detectColor(colorSensor1);
        drumConfiguration[2] = detectColor(colorSensor2);
    }


    private BallColor detectColor(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        // Purple = больше blue и red, меньше green
        if (blue > COLOR_THRESHOLD && red > COLOR_THRESHOLD && blue > green) {
            return BallColor.PURPLE;
        }

        else if (green > COLOR_THRESHOLD && green > blue && green > red) {
            return BallColor.GREEN;
        }

        return BallColor.UNKNOWN;
    }


    public boolean isValidConfiguration() {
        int purpleCount = 0;
        int greenCount = 0;

        for (BallColor color : drumConfiguration) {
            if (color == BallColor.PURPLE) purpleCount++;
            if (color == BallColor.GREEN) greenCount++;
        }

        return (purpleCount == 2 && greenCount == 1);
    }

    /**
     * Выполняет умную сортировку по команде от Vision
     */
    public void executeSmartSequence(ShootSequence sequence) {
        if (!isValidConfiguration()) {
            executeDefaultSequence();
            return;
        }

        // Находим индексы фиолетовых и зеленого мяча
        ArrayList<Integer> purpleSlots = new ArrayList<>();
        int greenSlot = -1;

        for (int i = 0; i < 3; i++) {
            if (drumConfiguration[i] == BallColor.PURPLE) {
                purpleSlots.add(i);
            } else if (drumConfiguration[i] == BallColor.GREEN) {
                greenSlot = i;
            }
        }

        int[] shootOrder = new int[3];

        switch (sequence) {
            case PPG: // Purple → Purple → Green
                shootOrder[0] = purpleSlots.get(0);
                shootOrder[1] = purpleSlots.get(1);
                shootOrder[2] = greenSlot;
                break;

            case PGP: // Purple → Green → Purple
                shootOrder[0] = purpleSlots.get(0);
                shootOrder[1] = greenSlot;
                shootOrder[2] = purpleSlots.get(1);
                break;

            case GPP: // Green → Purple → Purple
                shootOrder[0] = greenSlot;
                shootOrder[1] = purpleSlots.get(0);
                shootOrder[2] = purpleSlots.get(1);
                break;
        }

        executeShootOrder(shootOrder);
    }


    public void executeDefaultSequence() {
        executeShootOrder(new int[]{0, 1, 2});
    }


    private void executeShootOrder(int[] order) {
        for (int slot : order) {
            pushBall(slot);
            // Здесь можно добавить задержку или ждать подтверждения от shooter +++++++++++++++++++++++++++++++++++++
            retractServo(slot);
        }
    }

    /**
     * Выталкивает мяч из конкретного слота
     */
    public void pushBall(int slot) {
        switch (slot) {
            case 0:
                servo0.setPosition(SERVO_PUSH);
                break;
            case 1:
                servo1.setPosition(SERVO_PUSH);
                break;
            case 2:
                servo2.setPosition(SERVO_PUSH);
                break;
        }
    }

    /**
     * Возвращает servo в исходное положение
     */
    public void retractServo(int slot) {
        switch (slot) {
            case 0:
                servo0.setPosition(SERVO_RETRACT);
                break;
            case 1:
                servo1.setPosition(SERVO_RETRACT);
                break;
            case 2:
                servo2.setPosition(SERVO_RETRACT);
                break;
        }
    }

    /**
     * Возвращает все servo в исходное положение
     */
    public void retractAll() {
        servo0.setPosition(SERVO_RETRACT);
        servo1.setPosition(SERVO_RETRACT);
        servo2.setPosition(SERVO_RETRACT);
    }

    /**
     * Получить текущую конфигурацию барабана (для телеметрии)
     */
    public String getConfigurationString() {
        StringBuilder sb = new StringBuilder();
        for (BallColor color : drumConfiguration) {
            if (color == BallColor.PURPLE) sb.append("P");
            else if (color == BallColor.GREEN) sb.append("G");
            else sb.append("?");
        }
        return sb.toString();
    }

    /**
     * Получить цвет в конкретном слоте
     */
    public BallColor getColorAt(int slot) {
        if (slot >= 0 && slot < 3) {
            return drumConfiguration[slot];
        }
        return BallColor.UNKNOWN;
    }
}