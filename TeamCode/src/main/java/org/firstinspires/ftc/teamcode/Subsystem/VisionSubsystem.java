package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * VisionSubsystem: Manages AprilTag detection with alliance-aware filtering.
 * Supports dual detection modes: alliance-specific targeting and best-tag detection.
 * Based on reference implementation from FTC-25444-Code.
 */
public class VisionSubsystem {

    public enum AllianceColor {
        RED,
        BLUE
    }

    // AprilTag IDs for each alliance
    public static final int RED_ALLIANCE_TAG = 24;
    public static final int BLUE_ALLIANCE_TAG = 20;  // Using 20 for your field config

    // Vision components
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // Alliance configuration
    private AllianceColor currentAlliance;
    private int targetTagId;

    // Lifecycle state
    private boolean active = false;

    // Detection caching (prevents multiple queries per loop - mentor's optimization)
    private AprilTagDetection cachedTargetDetection = null;
    private long cacheTimestamp = 0;
    private static final long CACHE_VALIDITY_MS = 20;  // 50Hz update rate

    // Persistence filtering (mentor's approach - eliminates flickering)
    private static final int FRAMES_TO_CONFIRM = 3;  // 3 consecutive frames needed
    private int consecutiveDetections = 0;
    private int consecutiveLosses = 0;
    private boolean stableTagVisible = false;

    // Cached detection data (for backward compatibility with update() method)
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
        this.targetTagId = (alliance == AllianceColor.RED) ? RED_ALLIANCE_TAG : BLUE_ALLIANCE_TAG;
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

        // Auto-start (for backward compatibility)
        this.active = true;
    }

    /**
     * Starts the vision system. Call this after initialization.
     */
    public void start() {
        active = true;
    }

    /**
     * Stops the vision system and closes the portal.
     */
    public void stop() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
            visionPortal.close();
        }
        active = false;
    }

    /**
     * @return true if the vision system is active and running
     */
    public boolean isActive() {
        return active;
    }

    /**
     * Sets the alliance color. Changes which AprilTag we target.
     * @param alliance The alliance color (RED or BLUE)
     */
    public void setAlliance(AllianceColor alliance) {
        this.currentAlliance = alliance;
        this.targetTagId = (alliance == AllianceColor.RED) ? RED_ALLIANCE_TAG : BLUE_ALLIANCE_TAG;

        // Invalidate cache when switching alliance
        cachedTargetDetection = null;
        cacheTimestamp = 0;

        // Reset persistence filter
        resetPersistenceFilter();
    }

    /**
     * @return Current alliance color as a string ("RED" or "BLUE")
     */
    public String getAllianceColor() {
        return (targetTagId == RED_ALLIANCE_TAG) ? "RED" : "BLUE";
    }

    /**
     * @return The current alliance's target tag ID
     */
    public int getTargetTagId() {
        return targetTagId;
    }

    /**
     * Updates cached vision data. Call this every loop for backward compatibility.
     * Note: Direct getter methods (getTargetTag, getBestTag, etc.) query live data.
     * Uses persistence filtering for stable detection.
     */
    public void update() {
        // Use cached detection with persistence filtering
        AprilTagDetection target = getTargetTagCached();

        if (target != null && target.ftcPose != null) {
            hasTarget = true;
            yawDegrees = target.ftcPose.yaw;
            distanceCm = target.ftcPose.range * 2.54;
        } else {
            hasTarget = false;
            yawDegrees = Double.NaN;  // NaN instead of 0.0 (more correct)
            distanceCm = Double.NaN;
        }
    }

    // ===== ALLIANCE-SPECIFIC TARGET METHODS =====

    /**
     * Internal: Gets the alliance-specific target tag with caching.
     * Prevents multiple queries to AprilTagProcessor per loop cycle.
     * @return AprilTagDetection of alliance target, or null if not detected
     */
    private AprilTagDetection getTargetTagCached() {
        long now = System.currentTimeMillis();

        // Return cached value if fresh (within 20ms)
        if (cachedTargetDetection != null && (now - cacheTimestamp) < CACHE_VALIDITY_MS) {
            return cachedTargetDetection;
        }

        // Update cache
        cachedTargetDetection = null;
        cacheTimestamp = now;

        if (!active) return null;

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.id == targetTagId && d.ftcPose != null) {
                cachedTargetDetection = d;
                break;
            }
        }

        return cachedTargetDetection;
    }

    /**
     * Gets the alliance-specific target tag (live query, no caching).
     * For most uses, prefer hasTargetTag() or getTargetYaw() which use caching.
     * @return AprilTagDetection of alliance target, or null if not detected
     */
    public AprilTagDetection getTargetTag() {
        return getTargetTagCached();
    }

    /**
     * Checks if alliance-specific target tag is visible with persistence filtering.
     *
     * Persistence Filter (from mentor's approach):
     * - Requires 3 consecutive frames to confirm detection
     * - Requires 3 consecutive frames to confirm loss
     * - Eliminates single-frame flickering at 400-500cm distances
     * - ~60ms delay at 50Hz loop rate (acceptable for stability)
     *
     * @return true if tag is stably visible
     */
    public boolean hasTargetTag() {
        boolean rawDetection = getTargetTagCached() != null;

        // Update counters based on raw detection
        if (rawDetection) {
            consecutiveDetections++;
            consecutiveLosses = 0;
        } else {
            consecutiveLosses++;
            consecutiveDetections = 0;
        }

        // Switch to "visible" only after 3 consecutive detections
        if (consecutiveDetections >= FRAMES_TO_CONFIRM) {
            stableTagVisible = true;
        }

        // Switch to "not visible" only after 3 consecutive losses
        if (consecutiveLosses >= FRAMES_TO_CONFIRM) {
            stableTagVisible = false;
        }

        return stableTagVisible;
    }

    /**
     * Checks if tag is currently detected (raw, no persistence filtering).
     * Use this for testing/debugging. For production, use hasTargetTag().
     * @return true if tag detected in current frame
     */
    public boolean hasTargetTagRaw() {
        return getTargetTagCached() != null;
    }

    /**
     * Alias for hasTargetTag() - checks stable visibility.
     * @return true if alliance tag is stably visible
     */
    public boolean hasStableTarget() {
        return hasTargetTag();
    }

    /**
     * Resets persistence filter (for testing or alliance changes).
     */
    private void resetPersistenceFilter() {
        consecutiveDetections = 0;
        consecutiveLosses = 0;
        stableTagVisible = false;
    }

    /**
     * Gets yaw to alliance target tag (uses cached detection).
     * @return Yaw in degrees, or Double.NaN if not visible
     */
    public double getTargetYaw() {
        AprilTagDetection tag = getTargetTagCached();
        if (tag == null || tag.ftcPose == null) return Double.NaN;
        return tag.ftcPose.yaw;
    }

    /**
     * Gets distance to alliance target tag (uses cached detection).
     * @return Distance in centimeters, or Double.NaN if not visible
     */
    public double getTargetDistance() {
        AprilTagDetection tag = getTargetTagCached();
        if (tag == null || tag.ftcPose == null) return Double.NaN;
        return tag.ftcPose.range * 2.54;  // inches to cm
    }

    // ===== BEST TAG (ANY ALLIANCE) METHODS =====

    /**
     * Gets the closest AprilTag regardless of alliance (live query).
     * @return AprilTagDetection of closest tag, or null if none detected
     */
    public AprilTagDetection getBestTag() {
        if (!active) return null;

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
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
     * @return true if any AprilTag is visible (regardless of alliance)
     */
    public boolean hasTarget() {
        return getBestTag() != null;
    }

    /**
     * Alias for hasTarget() - checks if any AprilTag is visible.
     * @return true if any AprilTag is visible (regardless of alliance)
     */
    public boolean hasAnyTarget() {
        return hasTarget();
    }

    /**
     * @return ID of the closest AprilTag, or -1 if none detected
     */
    public int getBestTagId() {
        AprilTagDetection tag = getBestTag();
        if (tag == null) return -1;
        return tag.id;
    }

    /**
     * @return Distance in inches to the closest AprilTag, or -1.0 if none detected
     */
    public double getBestTagRange() {
        AprilTagDetection tag = getBestTag();
        if (tag == null || tag.ftcPose == null) return -1.0;
        return tag.ftcPose.range;
    }

    /**
     * @return Yaw error in degrees to the closest AprilTag, or Double.NaN if none detected
     */
    public double getYawError() {
        AprilTagDetection tag = getBestTag();
        if (tag == null || tag.ftcPose == null) return Double.NaN;
        return tag.ftcPose.yaw;
    }

    // ===== CACHED DATA GETTERS (for backward compatibility) =====

    /**
     * @return true if the correct AprilTag for current alliance was detected in last update()
     * @deprecated Use hasTargetTag() for live data
     */
    public boolean hasCachedTarget() {
        return hasTarget;
    }

    /**
     * @return Cached yaw in degrees from last update()
     * @deprecated Use getTargetYaw() for live data
     */
    public double getYawDegrees() {
        return yawDegrees;
    }

    /**
     * @return Cached distance in centimeters from last update()
     * @deprecated Use getTargetDistance() for live data
     */
    public double getDistanceCm() {
        return distanceCm;
    }

    /**
     * Closes the vision portal. Call this in opmode stop().
     */
    public void close() {
        stop();
    }

    // ===== SIMPLE TURRET WRAPPER API =====

    /**
     * Simple wrapper for turret: gets yaw angle to target
     * Convention: positive yaw = tag is to RIGHT of camera center
     * Uses alliance-specific target tag for more accurate aiming
     * @return yaw angle in degrees, or Double.NaN if no target
     */
    public double getYaw() {
        return getTargetYaw();  // Use cached version with NaN handling
    }
}
