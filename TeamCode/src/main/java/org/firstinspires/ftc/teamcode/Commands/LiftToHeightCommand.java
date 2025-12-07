package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

public class LiftToHeightCommand extends CommandBase {
    private final Lift lift;
    private final int targetHeight;

    public LiftToHeightCommand(Lift lift, int targetHeight){
        this.lift = lift;
        this.targetHeight = targetHeight;
        addRequirements(lift);
    }

    @Override
    public void initialize(){
        lift.setTicks(targetHeight);
    }

    @Override
    public void execute(){}

    @Override
    public boolean isFinished(){
        return Math.abs(lift.getCurrentPosition() - targetHeight) < 10;
    }

    @Override
    public void end(boolean interrupted){
        lift.lift(0);
    }


}
