package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class GetDistance extends CommandBase {

    private final Vision vision;
    private final Telemetry telemetry;

    public GetDistance(Vision vision, Telemetry telemetry) {
        this.vision = vision;
        this.telemetry = telemetry;

        addRequirements(vision);
    }

    @Override
    public void initialize() {
        telemetry.addLine("Looking for AprilTags...");
        telemetry.update();
    }

    @Override
    public void execute() {
        AprilTagDetection detection = vision.getFirstDetection();

        if (detection != null) {

            telemetry.addLine("TAG FOUND");
            telemetry.addData("ID", detection.id);

            if (detection.ftcPose != null) {

                double distanceInches = detection.ftcPose.y;   // Forward distance
                double distanceMM = distanceInches * 25.4;

                telemetry.addData("Distance (in)", distanceInches);
                telemetry.addData("Distance (mm)", distanceMM);

                // Optional offset calculation
                double tagCenterX = (detection.corners[0].x + detection.corners[1].x +
                        detection.corners[2].x + detection.corners[3].x) / 4.0;
                double imageCenterX = 640 / 2.0;
                double offset = tagCenterX - imageCenterX;

                telemetry.addData("Offset (px)", offset);

            } else {
                telemetry.addLine("Pose not available (ftcPose is null)");
                telemetry.addLine("Move closer or increase tag size");
            }

        } else {
            telemetry.addLine("NO TAG DETECTED");
        }

        telemetry.update();
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
