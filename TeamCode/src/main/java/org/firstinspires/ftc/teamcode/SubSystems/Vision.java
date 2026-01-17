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

    // ID тегов для последовательностей
    public static final int TAG_PPG = 101;
    public static final int TAG_PGP = 102;
    public static final int TAG_GPP = 103;

    // Пороги расстояния для Hood (в метрах) - КАЛИБРОВАТЬ!
    public static final double RANGE_CLOSE = 1.0;   // < 1м = CLOSE
    public static final double RANGE_FAR = 2.0;     // > 2м = FAR
    // между ними = MIDDLE

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

    /**
     * Определяет последовательность стрельбы по ID тега
     */
    public Sorter.ShootSequence getShootSequence() {
        int tagId = getBestTagId();

        switch (tagId) {
            case TAG_PPG:
                return Sorter.ShootSequence.PPG;
            case TAG_PGP:
                return Sorter.ShootSequence.PGP;
            case TAG_GPP:
                return Sorter.ShootSequence.GPP;
            default:
                return null;
        }
    }

    /**
     * Определяет позицию Hood на основе расстояния до тега
     */
    public Shooter.HoodPosition getHoodPosition() {
        double range = getBestTagRange();

        if (range < 0) {
            return null; // Тег не найден
        }

        if (range < RANGE_CLOSE) {
            return Shooter.HoodPosition.CLOSE;
        } else if (range > RANGE_FAR) {
            return Shooter.HoodPosition.FAR;
        } else {
            return Shooter.HoodPosition.MIDDLE;
        }
    }

    /**
     * Проверяет, найден ли тег с известной последовательностью
     */
    public boolean hasValidSequenceTag() {
        int tagId = getBestTagId();
        return (tagId == TAG_PPG || tagId == TAG_PGP || tagId == TAG_GPP);
    }

    public double getYawError() {
        AprilTagDetection tag = getBestTag();
        if (tag == null || tag.ftcPose == null) return Double.NaN;
        return tag.ftcPose.yaw;
    }

    public boolean hasTarget() {
        return getBestTag() != null;
    }
}