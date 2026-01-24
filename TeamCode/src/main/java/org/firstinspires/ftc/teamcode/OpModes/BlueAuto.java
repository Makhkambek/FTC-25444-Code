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

@Autonomous(name="Blue Auto", group="Autonomous")
public class BlueAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    private PathChain path1, path2, path3, path4;
    private final Pose startPose = new Pose(39.945, 135.779, Math.toRadians(270));

    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Vision vision;
    private Localizer localizer;

    public void buildPaths() {
        // Path 1: BezierLine (39.945, 135.779) -> (60.0, 84.0)
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(39.945, 135.779),
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();

        // Path 2: BezierCurve (60.0, 84.0) -> (55.034, 55.046) -> (16.262, 58.841)
        path2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(55.034, 55.046),
                                new Pose(16.262, 58.841)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 3: BezierCurve (16.262, 58.841) -> (62.117, 66.660) -> (60.0, 84.0)
        path3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(16.262, 58.841),
                                new Pose(62.117, 66.660),
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                .build();

        // Path 4: BezierCurve (60.0, 84.0) -> (61.915, 66.612) -> (16.503, 58.841)
        path4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(61.915, 66.612),
                                new Pose(16.503, 58.841)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(150))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Запуск Path 1 (turret направлен)
                shooter.on();
                follower.followPath(path1, true);
                setPathState(1);
                break;

            case 1: // Ожидание завершения Path 1
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    shooter.startShoot();
                    setPathState(2);
                }
                break;

            case 2: // Запуск Path 2 с intake
                intake.on();
                follower.followPath(path2, true);
                setPathState(3);
                break;

            case 3: // Ожидание завершения Path 2
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    setPathState(4);
                }
                break;

            case 4: // Запуск Path 3
                follower.followPath(path3, true);
                setPathState(5);
                break;

            case 5: // Ожидание завершения Path 3
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    setPathState(6);
                }
                break;

            case 6: // Ожидание 0.3 секунды
                if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                    shooter.startShoot();
                    setPathState(7);
                }
                break;

            case 7: // Запуск Path 4 с intake
                intake.on();
                follower.followPath(path4, true);
                setPathState(8);
                break;

            case 8: // Ожидание завершения Path 4 - завершение
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intake.off();

                    // Сохраняем финальную позицию робота для передачи в TeleOp
                    // Измените на фактические координаты конца автономки
                    localizer.setPosition(16.503, 58.841, 150);

                    setPathState(9);
                }
                break;

            case 9: // Завершено
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
        turret = new Turret(hardwareMap, vision);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        pathTimer = new Timer();

        buildPaths();
    }

    @Override
    public void start() {
        vision.start();
        pathTimer.resetTimer();

        turret.setGoalByAlliance(false);

        // Устанавливаем позицию турели (зафиксирована на всю автономку)
        turret.setTargetPosition(100);

        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        localizer.update();
        shooter.updateFSM(intake);
        shooter.updateHoodDynamic(turret);
        turret.holdPosition(); // Держит турель на установленной позиции
        autonomousPathUpdate();
    }

    @Override
    public void stop() {
        intake.off();
        shooter.off();
        turret.stop();
        vision.stop();
    }
}
