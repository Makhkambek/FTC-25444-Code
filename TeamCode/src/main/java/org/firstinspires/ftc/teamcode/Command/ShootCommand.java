package org.firstinspires.ftc.teamcode.Command;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.GateServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.IntakeSubsystem;

public class ShootCommand {

    private final ShooterSubsystem shooter;
    private final GateServoSubsystem gate;
    private final IntakeSubsystem intake;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean isShooting = false;

    public ShootCommand(ShooterSubsystem shooter, GateServoSubsystem gate, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.gate = gate;
        this.intake = intake;
    }

    // Call this when right bumper is pressed
    public void start() {
        if (!isShooting) {
            isShooting = true;
            shooter.setMaxSpeed();   // ramp to max
            timer.reset();
        }
    }

    // Call this every loop
    public void update() {
        if (!isShooting) return;

        // Wait a short moment for shooter to ramp up before activating intake and servo
        if (timer.seconds() >= 0.2 && timer.seconds() < 4.2) {
            intake.intakeIn();
            gate.open();
        }
        else if (timer.seconds() >= 4.2) {
            // Reset everything
            intake.stop();
            gate.close();
            shooter.setDefaultSpeed();
            isShooting = false;
        }
    }
}
