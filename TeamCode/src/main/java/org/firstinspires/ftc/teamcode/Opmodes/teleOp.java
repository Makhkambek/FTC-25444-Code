package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "CustomTeleOp", group = "TeleOp")
public class teleOp extends OpMode {

    private Follower follower;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private boolean headingLocked = false;
    private double targetHeading = 0;
    private final double kP_heading = 0.012;

    private Lift lift;
    private Outtake outtake;
    private Intake intake;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();

        lift = new Lift(hardwareMap);
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap, lift, outtake);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        if (gamepad1.right_bumper && !headingLocked) {
            headingLocked = true;
            targetHeading = follower.getHeading();
        } else if (!gamepad1.right_bumper && headingLocked && Math.abs(gamepad1.right_stick_x) > 0.01) {
            headingLocked = false;
        }

        double turn;
        if (headingLocked) {
            double error = targetHeading - follower.getHeading();
            while (error > Math.PI) error -= 2 * Math.PI;
            while (error < -Math.PI) error += 2 * Math.PI;
            turn = error * kP_heading;
        } else {
            turn = -gamepad1.right_stick_x;
        }

        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;

        if (slowMode) {
            forward *= slowModeMultiplier;
            strafe *= slowModeMultiplier;
            turn *= slowModeMultiplier;
        }

        follower.setTeleOpDrive(forward, strafe, turn, true);

        if (gamepad1.left_bumper) slowMode = !slowMode;

        if (gamepad2.dpad_up) lift.setState(Lift.State.HIGH);
        else if (gamepad2.dpad_right) lift.setState(Lift.State.MID);
        else if (gamepad2.dpad_left) lift.setState(Lift.State.HANG);
        else if (gamepad2.dpad_down) lift.setState(Lift.State.GND);

        lift.update();

        if (gamepad2.left_bumper) intake.setState(Intake.State.INTAKE_RUN);
        else if (gamepad2.right_bumper) intake.setState(Intake.State.OUTTAKE_RUN);
        else if (gamepad2.a) intake.setState(Intake.State.OPEN);
        else if (gamepad2.b) intake.setState(Intake.State.CLOSED);
        else if (gamepad2.x) intake.setState(Intake.State.TRANSFERRING);
        else if (gamepad2.y) intake.setState(Intake.State.SPECIMEN);
        else if (gamepad2.back) intake.setState(Intake.State.INITIAL);
        else if (gamepad2.start) intake.setState(Intake.State.OPENER);
        else intake.setState(Intake.State.IDLE);

        intake.update();

        if (gamepad2.left_trigger > 0.1) outtake.setState(Outtake.State.GRAB);
        else if (gamepad2.right_trigger > 0.1) outtake.setState(Outtake.State.SCORE);
        else outtake.setState(Outtake.State.IDLE);

        outtake.update();

        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getHeading()));
        telemetry.addData("Heading Locked", headingLocked);
        telemetry.addData("Slow Mode", slowMode);
        telemetry.addData("Lift Pos", lift.getCurrentPosition());
        telemetry.addData("Intake State", intake.getState());
        telemetry.addData("Outtake State", outtake.getState());
        telemetry.update();
    }
}
