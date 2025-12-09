
package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.ArmIntake;

public class MoveDoorCommand extends CommandBase {
    private final ArmIntake armIntake;
    private final int targetTicks;

    public MoveDoorCommand(ArmIntake armIntake, int targetTicks) {
        this.armIntake = armIntake;
        this.targetTicks = targetTicks;
        addRequirements(armIntake);
    }

    @Override
    public void initialize() {
        armIntake.setTargetPosition(targetTicks);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(armIntake.getTargetPosition() - armIntake.getCurrentPosition()) < 10;
    }

    @Override
    public void end(boolean interrupted) {
        armIntake.stop();
    }
}
