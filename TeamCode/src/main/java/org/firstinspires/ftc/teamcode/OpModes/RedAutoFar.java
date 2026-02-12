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

@Autonomous(name="Red AutoFar", group="Autonomous")
public class RedAutoFar extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path11, path33, path333;
    // Mirror: 144 - 26 = 118, 180 - 135 = 45
    private final Pose startPose = new Pose(87.914, 8.100, Math.toRadians(0));

    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Vision vision;
    private Localizer localizer;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87.914, 8.100),
                                new Pose(83.009, 42.290),
                                new Pose(124.928, 38.735)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))  // 180 - 135
                .build();

        // Mirror path2
        path2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(124.928, 38.735),
                                new Pose(83.061, 42.403),
                                new Pose(87.798, 8.951)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))  // 180 - 135, 180 - 180
                .build();

        // Mirror path3 - стреляет шестой
        path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.798, 8.951),

                                new Pose(128.061, 8.031)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path33 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.798, 8.951),

                                new Pose(128.061, 8.031)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
                .build();

        path333 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.798, 8.951),

                                new Pose(128.061, 8.031)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-20))
                .build();

        // Mirror path4 -
        path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(128.061, 8.031),

                                new Pose(87.805, 8.017)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Mirror path5 - девятый мяч
        path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.805, 8.017),

                                new Pose(100, 45)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

//        // Mirror path6 - едет за 12
//        path6 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(93.886, 80.616),
//
//                                new Pose(113.790, 80.656)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
//                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                turret.setTargetAngle(-70);
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
                    turret.setTargetAngle(-62);
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
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0 && shooter.isIdle()) {
                    intake.on();
                    follower.followPath(path3,0.6, true);
                    setPathState(33);
                }
                break;

            case 33: // Запуск Path 3
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    intake.on();
                    follower.followPath(path33,0.5, true);
                    setPathState(333);
                }
                break;

            case 333: // Запуск Path 3
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.5 ) {
                    intake.on();
                    follower.followPath(path333,0.5, true);
                    setPathState(4);
                }
                break;

            case 4: // Ожидание завершения Path 3
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 3.5 && shooter.isIdle()) {
                    follower.followPath(path4,0.8, true);
                    turret.setTargetAngle(-62);
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
                    setPathState(13);
                }
                break;


            case 13: // Завершено
                localizer.setPosition(100, 45, 90);
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
        vision.setAlliance(true); // Red alliance
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

        shooter.setHoodPosition(0.72);
        shooter.setTargetVelocity(1850);

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
