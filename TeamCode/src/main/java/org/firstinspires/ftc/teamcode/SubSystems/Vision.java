package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;

import java.util.List;

public class Vision {

    private VisionPortal portal;
    private AprilTagProcessor aprilTag;
    private boolean active = false;

    // ID тегов для разных альянсов
    public static final int RED_ALLIANCE_TAG = 11;   // AprilTag для красного альянса
    public static final int BLUE_ALLIANCE_TAG = 12;  // AprilTag для синего альянса

    private int targetTagId = RED_ALLIANCE_TAG; // По умолчанию красный

    public void init(HardwareMap hw) {
        aprilTag = new AprilTagProcessor.Builder().build();

        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTag)
                .build();
    }

    public void start() {
        active = true;
    }

    public void stop() {
        if (portal != null) {
            portal.stopStreaming();
            portal.close();
        }
        active = false;
    }

    public boolean isActive() {
        return active;
    }

    /**
     * Устанавливает целевой tag для отслеживания
     * @param isRedAlliance true для красного альянса, false для синего
     */
    public void setAlliance(boolean isRedAlliance) {
        targetTagId = isRedAlliance ? RED_ALLIANCE_TAG : BLUE_ALLIANCE_TAG;
    }

    /**
     * Получает целевой AprilTag (для текущего альянса)
     */
    public AprilTagDetection getTargetTag() {
        if (!active) return null;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.id == targetTagId && d.ftcPose != null) {
                return d;
            }
        }
        return null;
    }

    /**
     * Проверяет, виден ли целевой tag
     */
    public boolean hasTargetTag() {
        return getTargetTag() != null;
    }

    /**
     * Получает расстояние до целевого тега в сантиметрах
     */
    public double getTargetDistance() {
        AprilTagDetection tag = getTargetTag();
        if (tag == null || tag.ftcPose == null) return -1;
        return tag.ftcPose.range * 2.54; // дюймы в см
    }

    /**
     * Получает yaw ошибку для целевого тега (для turret)
     */
    public double getTargetYaw() {
        AprilTagDetection tag = getTargetTag();
        if (tag == null || tag.ftcPose == null) return Double.NaN;
        return tag.ftcPose.yaw;
    }

    public AprilTagDetection getBestTag() {
        if (!active) return null;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) return null;

        AprilTagDetection best = null;
        double bestRange = Double.MAX_VALUE;

        for (AprilTagDetection d : detections) {
            if (d.ftcPose == null) continue;

            if (d.ftcPose.range < bestRange) {
                bestRange = d.ftcPose.range;
                best = d;
            }
        }
        return best;
    }

    /**
     * Получает ID ближайшего тега
     */
    public int getBestTagId() {
        AprilTagDetection tag = getBestTag();
        if (tag == null) return -1;
        return tag.id;
    }

    /**
     * Получает дистанцию до ближайшего тега (в метрах)
     */
    public double getBestTagRange() {
        AprilTagDetection tag = getBestTag();
        if (tag == null) return -1;
        return tag.ftcPose.range;
    }


    public double getYawError() {
        AprilTagDetection tag = getBestTag();
        if (tag == null || tag.ftcPose == null) return Double.NaN;
        return tag.ftcPose.yaw;
    }

    public boolean hasTarget() {
        return getBestTag() != null;
    }

    // Геттеры для телеметрии
    public int getTargetTagId() {
        return targetTagId;
    }

    public String getAllianceColor() {
        return (targetTagId == RED_ALLIANCE_TAG) ? "RED" : "BLUE";
    }
}