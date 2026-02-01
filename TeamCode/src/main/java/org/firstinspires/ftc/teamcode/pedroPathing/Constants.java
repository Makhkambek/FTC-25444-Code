package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-45)
            .lateralZeroPowerAcceleration(-83.5)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.004,0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.6,0,0.005,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0,0.001,0.6,0.9))
            .centripetalScaling(0.0005)
            .mass(11);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(81.5)
            .yVelocity(62.5);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4.9)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // CRITICAL: These encoder directions MUST match Localizer.java!
            // Forward encoder: mounted parallel to robot front/back → controls Y-axis (vertical movement)
            // Strafe encoder: mounted perpendicular to robot → controls X-axis (horizontal movement)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)    // Y-axis (forward/backward)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);   // X-axis (left/right)

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}