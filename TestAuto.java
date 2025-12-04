package teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "TestAuto", group = "Autonomous")
public class TestAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private boolean poseSet = false;
    private int pathState = 0;
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8;
    private final Pose startPose = new Pose(7.722, 103.304, 0);
    private ElapsedTime timer = new ElapsedTime();

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(7.722, 103.304, Point.CARTESIAN),
                                new Point(25.670, 121.252, Point.CARTESIAN),
                                new Point(14.0, 128.261, Point.CARTESIAN)) // 16.487 and 126.261
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-35))
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();

        path2 = follower.pathBuilder() // take the second sample
                .addPath(
                        new BezierLine(
                                new Point(14.0, 128.261, Point.CARTESIAN),
                                new Point(35.0, 121, Point.CARTESIAN)) // 35
                )
                .setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(2.0)
                .build();

        path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(35.0, 121, Point.CARTESIAN),
                                new Point(13.0, 129.261, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-35))
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();

        path4 = follower.pathBuilder() // третий sample взять take the 3rd sample
                .addPath(
                        new BezierLine(
                                new Point(13.0, 129.261, Point.CARTESIAN),
                                new Point(34.435, 130.5, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();

        path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(34.435, 130.5, Point.CARTESIAN),
                                new Point(13.0, 129.261, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-35))
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();

        path6 = follower.pathBuilder() // for the last one
                .addPath(
                        new BezierCurve(
                                new Point(13.0, 129.261, Point.CARTESIAN),
                                new Point(20.661, 121.043, Point.CARTESIAN),
                                new Point(42.5, 131.5, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(90))
                .build();

        path7 = follower.pathBuilder() // go back after last one
                .addPath(
                        new BezierLine(
                                new Point(42.5, 131.5, Point.CARTESIAN),
                                new Point(13.0, 129.261, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-30))
                .setZeroPowerAccelerationMultiplier(1.0)
                .build();
        //
        path8 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(13.0, 129.261, Point.CARTESIAN),
                                new Point(55.096, 127.513, Point.CARTESIAN),
                                new Point(60.809, 95.748, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(270))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1, true);
                timer.reset();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && timer.seconds() >= 1.5) {
                    timer.reset();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && timer.seconds() > 1.2) {
                    follower.followPath(path2, true);
                    timer.reset();
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy() && timer.seconds() > 2.5) {
                    timer.reset();
                    setPathState(4);
                }
                break;

            case 4:
                if (timer.seconds() > 1.0 && !follower.isBusy()) {
                    timer.reset();
                    setPathState(5);
                }
                break;
            case 5:
                if (timer.seconds() >= 1.5 && !follower.isBusy()) {
                    follower.setMaxPower(0.9);
                    follower.followPath(path3, true);
                    timer.reset();
                    setPathState(6);
                }
                break;

            case 6:
                if (timer.seconds() >= 2.0 && !follower.isBusy()) {
                    timer.reset();
                    setPathState(7);
                }
                break;

            case 7: // поехал за третьим сэмплом went for the 3rd sample
                if (!follower.isBusy() && timer.seconds() > 1.0) {
                    follower.setMaxPower(1);
                    follower.followPath(path4, true);
                    timer.reset();
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && timer.seconds() > 3.0) {
                    timer.reset();
                    setPathState(9);
                }
                break;
            case 9:
                if (timer.seconds() > 1.0 && !follower.isBusy()) {
                    timer.reset();
                    setPathState(10);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}
