package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Subsystems.*;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends CommandOpMode {

    private Drivetrain drive;
    private HangIntake hang;
    private ArmIntake armIntake;
    private Clutch clutch;
    private Vision vision;
    private Push push;

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    @Override
    public void initialize() {
        // -------- Subsystems --------
        drive = new Drivetrain(hardwareMap);
        hang = new HangIntake(hardwareMap);
        armIntake = new ArmIntake(hardwareMap);
        clutch = new Clutch(hardwareMap);
        vision = new Vision(hardwareMap);
        push = new Push(hardwareMap);

        // -------- Gamepads --------
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // -------- Drive --------
        drive.setDefaultCommand(
                new DriveCommand(
                        drive,
                        () -> {
                            double y = -gamepadEx1.getLeftY();
                            return Math.abs(y) > 0.05 ? y : 0.0;
                        },
                        () -> {
                            double y = -gamepadEx1.getRightY();
                            return Math.abs(y) > 0.05 ? y : 0.0;
                        }
                )
        );

        // -------- Vision (DetectTagCommand) --------
        gamepadEx1.getGamepadButton(GamepadKeys.Button.X)
                .whileHeld(new DetectTagCommand(vision, telemetry, gamepad1));

        // -------- HangIntake (Gamepad2: B = Up, X = Down) --------
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(new HangIntakeCommand(hang, 1.0));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.X)
                .whileHeld(new HangIntakeCommand(hang, -1.0));

        // -------- ArmIntake (Gamepad2: A = Up, Y = Down) --------
        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(new ArmIntakeCommand(armIntake, 1.0));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(new ArmIntakeCommand(armIntake, -1.0));

        // -------- Door (Gamepad2 D-Pad) --------
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new MoveDoorCommand(armIntake, 0));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new MoveDoorCommand(armIntake, 150));

        // -------- Clutch (Gamepad2 bumpers) --------
        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(clutch::open);
        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(clutch::close);

        // -------- Push (Gamepad2 triggers) --------
        new Trigger(() -> gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(new OpenPushCommand(push, clutch));

        new Trigger(() -> gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(new ClosePushCommand(push, clutch));

        telemetry.addLine("TeleOp initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }


}