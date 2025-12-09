package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase{
    private Drivetrain drive;
    private DoubleSupplier leftSupplier;
    private DoubleSupplier rightSupplier;

    public DriveCommand(Drivetrain drive, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier){
        this.drive = drive;
        this.leftSupplier = leftSupplier;
        this.rightSupplier = rightSupplier;

        addRequirements(drive);
    }

    @Override
    public void execute(){
        drive.arcadeDrive(leftSupplier.getAsDouble(), rightSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted){
        drive.stop();
    }

    public boolean isFinished(){
        return false;
    }
}