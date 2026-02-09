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
public class RedAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;
    private final Pose startPose = new Pose(118, 129, Math.toRadians(43));

    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Vision vision;
    private Localizer localizer;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(
                        new BezierLine(
                                new Pose(118.000, 129.000),

                                new Pose(90.554, 99.886)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(43))
                .build();

        path2 = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(
                        new BezierCurve(
                                new Pose(90.554, 99.886),
                                new Pose(91.033, 69.166),
                                new Pose(97.680, 53.238),
                                new Pose(117.024, 55.100)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();

        path3 = follower.pathBuilder() //стреляет шестой
                .setGlobalDeceleration()
                .addPath(
                        new BezierCurve(
                                new Pose(117.024, 55.100),
                                new Pose(90.658, 67.362),
                                new Pose(93.821, 83.920)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path4 = follower.pathBuilder() //едет за девятым
                .setGlobalDeceleration()
                .addPath(
                        new BezierCurve(
                                new Pose(93.821, 83.920),
                                new Pose(100.772, 65.779),
                                new Pose(121.252, 61.946)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                .build();

        path5 = follower.pathBuilder() //девятый мяч
                .setGlobalDeceleration()
                .addPath(
                        new BezierCurve(
                                new Pose(121.252, 61.946),
                                new Pose(97.974, 67.501),
                                new Pose(93.770, 84.100)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(0))
                .build();

        path6 = follower.pathBuilder() //едет за 12
                .setGlobalDeceleration()
                .addPath(
                        new BezierCurve(
                                new Pose(93.770, 84.100),
                                new Pose(102.504, 64.360),
                                new Pose(121.252, 62.150)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                .build();

        path7 = follower.pathBuilder() //стреляет 12
                .setGlobalDeceleration()
                .addPath(
                        new BezierCurve(
                                new Pose(121.252, 62.150),
                                new Pose(103.369, 65.658),
                                new Pose(93.750, 83.728)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(0))
                .build();

        path8 = follower.pathBuilder() //едет за 15
                .setGlobalDeceleration()
                .addPath(
                        new BezierLine(
                                new Pose(93.750, 83.728),

                                new Pose(114.182, 83.571)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path9 = follower.pathBuilder() //стреляет 15
                .setGlobalDeceleration()
                .addPath(
                        new BezierLine(
                                new Pose(114.182, 83.571),

                                new Pose(93.832, 84.014)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1, true);
//                follower.followPath(path1, 0.6,true);
                setPathState(50);

                break;

            case 50:
                if (!follower.isBusy() ) {
                    shooter.startShoot();
                    setPathState(1);
                }
                break;

            case 1: // Ожидание завершения Path 1
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.5) {
                    intake.on();
                    follower.followPath(path2, true);
                    turret.setTargetAngle(-55);
                    setPathState(2);
                }
                break;

            case 2: //
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
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
                    follower.followPath(path4, true);
                    setPathState(5);
                }
                break;

            case 5: // Ожидание завершения Path 4 - завершение
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 3.5) {
                    follower.followPath(path5, true);
//                    localizer.setPosition(16.503, 58.841, 150);

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

            case 7: // Ожидание завершения Path 3
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5 && shooter.isIdle()) {
                    intake.on();
                    follower.followPath(path6, true);
                    setPathState(8);
                }
                break;

            case 8: // Ожидание завершения Path 4 - завершение
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 3.5) {
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

            case 10: // Ожидание завершения Path 3
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5 && shooter.isIdle()) {
                    intake.on();
                    follower.followPath(path8, true);
                    setPathState(11);
                }
                break;

            case 11: // Ожидание завершения Path 4 - завершение
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    follower.followPath(path9, true);
                    setPathState(12);
                }
                break;

            case 12: // Запуск
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    intake.off();
                    shooter.startShoot();
                    setPathState(13);
                }
                break;



            case 13: // Завершено
                localizer.setPosition(16.503, 58.841, 150);
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

        shooter.setHoodPosition(0.42);
        shooter.setTargetVelocity(1450.0);

        turret.setTargetAngle(0.0);

        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        localizer.update();
        turret.autoAim();
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
