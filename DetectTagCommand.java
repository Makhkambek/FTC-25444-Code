package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DetectTagCommand extends CommandBase {
    private final Vision vision;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private boolean hasRumbled;

    public DetectTagCommand(Vision vision, Telemetry telemetry, Gamepad gamepad) {
        this.vision = vision;
        this.telemetry = telemetry;
        this.gamepad = gamepad;

        addRequirements(vision);
    }

    @Override
    public void initialize() {
        telemetry.addLine("DTC started");
        telemetry.update();
        hasRumbled = false;
    }
    @Override
    public void execute(){
        AprilTagDetection detection = vision.getFirstDetection();

        if (detection != null) {
            double tagCenterX = (detection.corners[0].x + detection.corners[1].x +
                    detection.corners[2].x + detection.corners[3].x) / 4.0;

            double imageCenterX = 640 / 2.0;

            double offset = tagCenterX - imageCenterX; // pixels l(-) / r(+)
            telemetry.addData("Tag offset (px)", offset);

            // Rumble if aligned within ~20px
            if (Math.abs(offset) < 60 && !hasRumbled) {
                gamepad.rumble(1.0, 1.0, 500);
                hasRumbled = true;
            }

            // Metadata (if available in your SDK)
            if (detection.metadata != null) {
                telemetry.addData("Tag size (mm)", detection.metadata.tagsize);
                telemetry.addData("Field position", detection.metadata.fieldPosition);
            } else {
                telemetry.addLine("No pose metadata available");
            }
        } else {
            telemetry.addLine("No tag detected");
            hasRumbled = false;
        }

        telemetry.update();
    }

    @Override
    public void end(boolean interrupted) {
        telemetry.addLine("DTC ended");
        telemetry.update();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
