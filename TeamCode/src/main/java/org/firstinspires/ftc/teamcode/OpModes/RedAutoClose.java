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

@Autonomous(name="Red Auto", group="Autonomous")
public class RedAutoClose extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path11;
    // Mirror: 144 - 26 = 118, 180 - 135 = 45
    private final Pose startPose = new Pose(118.006, 128.797, Math.toRadians(45));

    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Vision vision;
    private Localizer localizer;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(118.006, 128.797),

                                new Pose(92.006, 100.739)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))  // 180 - 135
                .build();

        // Mirror path2
        path2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(92.006, 100.739),
                                new Pose(89.585, 51.681),
                                new Pose(121.755, 53.711)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))  // 180 - 135, 180 - 180
                .build();

        // Mirror path3 - стреляет шестой
        path3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(121.755, 53.711),
                                new Pose(98.054, 64.989),
                                new Pose(93.790, 80.549)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Mirror path4 - едет за девятым
        path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(93.790, 80.549),

                                new Pose(117.487, 80.539)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Mirror path5 - девятый мяч
        path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(117.487, 80.539),

                                new Pose(93.886, 80.616)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Mirror path6 - едет за 12
        path6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(93.886, 80.616),
                                new Pose(82.215, 29.421),
                                new Pose(125.258, 35.594)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path7 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(125.258, 35.594),
                                new Pose(91.585, 41.111),
                                new Pose(93.886, 80.626)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(93.886, 80.626),

                                new Pose(105, 80.656)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1, true);
                setPathState(50);

                break;

            case 50:
                if (!follower.isBusy() ) {
                    shooter.startShoot();
                    setPathState(1);
                }
                break;

            case 1: // едет брать 6
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.5) {
                    intake.on();
                    follower.followPath(path2,0.8, true);
                    turret.setTargetAngle(-55);  // Mirror: -55 instead of 55
                    setPathState(2);
                }
                break;

            case 2: //стреляет 6й
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
//                    intake.off();
                    follower.followPath(path3, true);
                    setPathState(3);
                }
                break;

            case 3: // Запуск Path 3
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
                         intake.off();
                        shooter.startShoot();
                    setPathState(4);
                }
                break;

            case 4: // Ожидание завершения Path 3
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5 && shooter.isIdle()) {
                    intake.on();
                    follower.followPath(path4,0.8, true);
                    setPathState(5);
                }
                break;

            case 5: // Ожидание завершения Path 4 - завершение
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    follower.followPath(path5, true);

                    setPathState(6);
                }
                break;

            case 6: // Запуск
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    intake.off();
                    shooter.startShoot();
                    setPathState(7);
                }
                break;


            case 7: // Ожидание завершения Path 4 - завершение
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5 && shooter.isIdle()) {
                    intake.on();
                    follower.followPath(path6, true);
//                    disableAutoAim = true;  // Stop autoAim from overwriting
//                    turret.setTargetAngle(0);
                    setPathState(8);
                }
                break;

            case 8: // Ожидание завершения Path 4 - завершение
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    follower.followPath(path7, true);

                    setPathState(9);
                }
                break;

            case 9: // Запуск
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    intake.off();
                    shooter.startShoot();
                    setPathState(10);
                }
                break;

            case 10: // Ожидание завершения Path 4 - завершение
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    follower.followPath(path8, true);
                    turret.setTargetAngle(0);
                    setPathState(13);
                }
                break;

            case 13: // Завершено
                localizer.setPosition(105, 80.656, 270);
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

        shooter.setHoodPosition(0.38);
        shooter.setTargetVelocity(1450.0);

        turret.setTargetAngle(0.0);

        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        localizer.update();
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
