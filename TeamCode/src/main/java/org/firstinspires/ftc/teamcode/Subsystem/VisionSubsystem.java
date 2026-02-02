package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * VisionSubsystem: Manages AprilTag detection with alliance-aware filtering.
 * Only tracks the AprilTag corresponding to the current alliance color.
 */
public class VisionSubsystem {

    public enum AllianceColor {
        RED,
        BLUE
    }

    // AprilTag IDs
    private static final int BLUE_TAG_ID = 20;
    private static final int RED_TAG_ID = 24;

    // Vision components
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;

    // Alliance configuration
    private AllianceColor currentAlliance;

    // Cached detection data
    private boolean hasTarget;
    private double yawDegrees;
    private double distanceCm;

    /**
     * Constructs a VisionSubsystem.
     * @param hardwareMap Hardware map from the opmode
     * @param alliance Initial alliance color (RED or BLUE)
     */
    public VisionSubsystem(HardwareMap hardwareMap, AllianceColor alliance) {
        this.currentAlliance = alliance;
        this.hasTarget = false;
        this.yawDegrees = 0.0;
        this.distanceCm = 0.0;

        // Initialize AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .build();

        // Initialize VisionPortal with webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    /**
     * Sets the alliance color. Changes which AprilTag we care about.
     * @param alliance The alliance color (RED or BLUE)
     */
    public void setAlliance(AllianceColor alliance) {
        this.currentAlliance = alliance;
    }

    /**
     * Updates vision data. Call this every loop to refresh detections.
     * Filters detections to only consider the tag matching current alliance.
     * If multiple valid tags exist, chooses the closest one.
     */
    public void update() {
        // Get current detections
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        // Determine target tag ID based on alliance
        int targetTagId = (currentAlliance == AllianceColor.BLUE) ? BLUE_TAG_ID : RED_TAG_ID;

        // Find the closest matching tag
        AprilTagDetection closestDetection = null;
        double closestDistance = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {
            if (detection.id == targetTagId && detection.ftcPose != null) {
                double range = detection.ftcPose.range; // in inches
                if (range < closestDistance) {
                    closestDistance = range;
                    closestDetection = detection;
                }
            }
        }

        // Update cached values
        if (closestDetection != null) {
            hasTarget = true;
            yawDegrees = closestDetection.ftcPose.yaw; // degrees, positive = right
            distanceCm = closestDetection.ftcPose.range * 2.54; // inches to cm
        } else {
            hasTarget = false;
            yawDegrees = 0.0;
            // Keep last known distance for safety, or reset to 0
            distanceCm = 0.0;
        }
    }

    /**
     * @return true if the correct AprilTag for current alliance is visible
     */
    public boolean hasTarget() {
        return hasTarget;
    }

    /**
     * @return Yaw in degrees from camera to target tag.
     *         Positive = tag is to the right, Negative = tag is to the left.
     *         Returns 0 if no target.
     */
    public double getYawDegrees() {
        return yawDegrees;
    }

    /**
     * @return Distance in centimeters from camera to target tag.
     *         Returns 0 if no target (or last known distance).
     */
    public double getDistanceCm() {
        return distanceCm;
    }

    /**
     * Closes the vision portal. Call this in opmode stop() if needed.
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}