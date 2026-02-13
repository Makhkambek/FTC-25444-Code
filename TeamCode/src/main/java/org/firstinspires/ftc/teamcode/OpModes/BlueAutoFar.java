package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.SubSystems.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Blue AutoFar", group="Autonomous")
public class BlueAutoFar extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;
    private final Pose startPose = new Pose(57.684, 8.357, Math.toRadians(-180));

    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Vision vision;
    private Localizer localizer;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(57.684, 8.357),

                                new Pose(17, 9.169)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                .build();

        path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(17, 9.169),

                                new Pose(57.684, 8.357)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                .build();



        path3 = follower.pathBuilder() //стреляет шестой
                .addPath(
                        new BezierCurve(
                                new Pose(57.681, 8.327),
                                new Pose(61.719, 39.423),
                                new Pose(19.480, 37.850)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                .build();

        path4 = follower.pathBuilder() //едет за девятым
                .addPath(
                        new BezierCurve(
                                new Pose(19.480, 37.850),
                                new Pose(61.821, 39.568),
                                new Pose(57.449, 10.740)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                .build();

        path5 = follower.pathBuilder() //девятый мяч
                .addPath(
                        new BezierLine(
                                new Pose(57.449, 10.740),

                                new Pose(44.539, 13.272)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(90))
                .build();

//        path6 = follower.pathBuilder() //едет за 12
//                .addPath(
//                        new BezierLine(
//                                new Pose(57.621, 22.477),
//
//                                new Pose(46.433, 22.616)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(90))
//                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                turret.setTargetAngle(70);
                if(pathTimer.getElapsedTimeSeconds() >= 1) {
                    shooter.startShoot();
                    setPathState(50);
                }
                break;

            case 50:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.3 && shooter.isIdle() ) {
                    intake.on();
                    follower.followPath(path1, 0.8, true);
                    setPathState(1);
                }
                break;

            case 1: // едет брать 6
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2) {
                    turret.setTargetAngle(70);
                    intake.off();
                    follower.followPath(path2,0.8, true);
                    setPathState(2);
                }
                break;

            case 2: //стреляет 6й
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    shooter.startShoot();
                    setPathState(3);
                }
                break;

            case 3: // Запуск Path 3
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0 && shooter.isIdle()) {
                    intake.on();
                    follower.followPath(path3,0.8, true);
                    setPathState(4);
                }
                break;

            case 4: // Ожидание завершения Path 3
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5 && shooter.isIdle()) {
                    follower.followPath(path4,0.8, true);
                    turret.setTargetAngle(70);
                    setPathState(5);
                }
                break;

            case 5: // Ожидание завершения Path 4 - завершение
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    intake.off();
                    shooter.startShoot();
                    setPathState(6);
                }
                break;

            case 6: // Запуск
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    follower.followPath(path5, true);
                    turret.setTargetAngle(0);
                    setPathState(7);
                }
                break;


            case 13: // Завершено
                localizer.setPosition(44.539, 13.272, 90);
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        localizer = Localizer.getInstance(hardwareMap);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        vision = new Vision();
        vision.init(hardwareMap);
        vision.setAlliance(false); // Blue alliance
        turret = new Turret(hardwareMap, vision, localizer);

        follower = Constants.createFollower(hardwareMap);

        // CRITICAL: Update Pinpoint ONCE before setting starting pose to initialize encoder data
        follower.update();

        follower.setStartingPose(startPose);

        pathTimer = new Timer();

        buildPaths();
    }

    @Override
    public void start() {
        vision.start();
        pathTimer.resetTimer();

        shooter.setHoodPosition(0.7);
        shooter.setTargetVelocity(1830);

        turret.setTargetAngle(0.0);

        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        localizer.update();
        // Only use fixed angles set by setTargetAngle(), no auto-aim
        turret.maintainTarget();
        shooter.updatePID();
        shooter.updateFSM(intake);
        autonomousPathUpdate();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.breakFollowing();
        }
        if (intake != null) {
            intake.off();
        }
        if (shooter != null) {
            shooter.off();
        }
        if (turret != null) {
            turret.stop();
        }
        if (vision != null) {
            vision.stop();
        }
    }
}
