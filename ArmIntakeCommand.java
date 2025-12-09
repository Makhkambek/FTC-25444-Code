package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.ArmIntake;

public class ArmIntakeCommand extends CommandBase {
    private final ArmIntake armIntake;
    private final double power;

    public  ArmIntakeCommand(ArmIntake armIntake, double power){
        this.power  = power;
        this.armIntake = armIntake;
        addRequirements(armIntake);
    }
    @Override
    public void initialize(){
        if (power > 0){
            armIntake.intakeOn();
        } else if (power < 0){
            armIntake.intakeReverse();
        }
        else {
            armIntake.intakeOff();
        }
    }
    @Override
    public void end(boolean interrupted){
      armIntake.intakeOff();
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
