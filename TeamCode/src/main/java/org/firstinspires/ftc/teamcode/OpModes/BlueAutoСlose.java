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

@Autonomous(name="Blue AutoClose", group="Autonomous")
public class BlueAutoСlose extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path50;
    private final Pose startPose = new Pose(26, 129, Math.toRadians(135));

    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Vision vision;
    private Localizer localizer;

    public void buildPaths() {
        path1 = follower.pathBuilder()  //стреляет первый 3
                .addPath(
                        new BezierLine(
                                new Pose(26.000, 129.000),

                                new Pose(47.922, 104.702)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        path2 = follower.pathBuilder() //едет за вторым
                .addPath(
                        new BezierCurve(
                                new Pose(47.922, 104.702),
                                new Pose(57.297, 60.042),
                                new Pose(43.464, 62.743)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        path50 = follower.pathBuilder() //подправляет второй
                .addPath(
                        new BezierLine(
                                new Pose(43.464, 62.743),

                                new Pose(8.712, 62.430)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();



        path3 = follower.pathBuilder() //едет стреляет 6 мячец!!
                .addPath(
                        new BezierCurve(
                                new Pose(8.712, 62.430),
                                new Pose(51.788, 71.802),
                                new Pose(46.102, 87.079)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path4 = follower.pathBuilder() //едет за 9 мячей
                .addPath(
                        new BezierCurve(
                                new Pose(46.102, 87.079),
                                new Pose(68.068, 27.477),
                                new Pose(10.663, 38.908)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();


        path5 = follower.pathBuilder() //едет стрелять 9й
                .addPath(
                        new BezierCurve(
                                new Pose(10.663, 38.908),
                                new Pose(58.291, 41.980),
                                new Pose(45.987, 87.079)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path6 = follower.pathBuilder() //едет за 12й
                .addPath(
                        new BezierLine(
                                new Pose(45.987, 87.079),

                                new Pose(20.693, 87.011)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path7 = follower.pathBuilder() //едет стрелять 12й
                .addPath(
                        new BezierLine(
                                new Pose(20.693, 87.011),

                                new Pose(45.975, 87.147)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path8 = follower.pathBuilder() //едет парковаться
                .addPath(
                        new BezierLine(
                                new Pose(45.975, 87.147),

                                new Pose(25.553, 71.819)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1, true);
                setPathState(50);

                break;

            case 50:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.5 ) {
                    shooter.startShoot();
                    setPathState(1);
                }
                break;

            case 1: // едет брать 6
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.5) {
                    intake.on();
                    follower.followPath(path2,0.8, true);
                    turret.setTargetAngle(48);
                    setPathState(100);
                }
                break;

            case 100: // едет брать 6
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.6) {
                    intake.on();
                    follower.followPath(path50,0.6, true);
                    turret.setTargetAngle(48);  // Mirror: -55 instead of 55
                    setPathState(2);
                }
                break;

            case 2: //стреляет 6й
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

            case 4: // едет за 9м
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5 && shooter.isIdle()) {
                    intake.on();
                    turret.setTargetAngle(48);
                    follower.followPath(path4,0.8, true);
                    setPathState(5);
                }
                break;

            case 5: // едет обратно стрелять 9й
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    follower.followPath(path5, true);

                    setPathState(6);
                }
                break;

            case 6: // стреляет 9й
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    intake.off();
                    shooter.startShoot();
                    setPathState(7);
                }
                break;

            case 7: //едет забрать 12й
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.5 && shooter.isIdle()) {
                    intake.on();
                    follower.followPath(path6, 0.6, true);
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
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.9) {
                    follower.followPath(path8, true);
                    turret.setTargetAngle(0);
                    setPathState(13);
                }
                break;


            case 13: // Завершено
                localizer.setPosition(25.553, 71.819, 270);
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

        follower.update();

        follower.setStartingPose(startPose);

        pathTimer = new Timer();

        buildPaths();
    }

    @Override
    public void start() {
        vision.start();
        pathTimer.resetTimer();

        shooter.setHoodPosition(0.41); //0.41
        shooter.setTargetVelocity(1350.0); //1350

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
